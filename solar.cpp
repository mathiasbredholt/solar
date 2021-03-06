#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <chrono>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <algorithm>

#include "tinyosc/tinyosc.h"
#include <arpa/inet.h>
#include <sys/socket.h>

const int BUFFER_SIZE = 1024;
const int PORT = 7700;
char buffer[BUFFER_SIZE];
int s;
void* addr_receiver;

const int NUM_LEDS = 3;

enum State {
    OFF,
    CIVIL,
    NAUTICAL,
    ASTRONIMICAL,
    AURORA
};

typedef struct RgbColor {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor {
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

RgbColor hsv2rgb(HsvColor hsv) {
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0) {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
    case 0:
        rgb.r = hsv.v; rgb.g = t; rgb.b = p;
        break;
    case 1:
        rgb.r = q; rgb.g = hsv.v; rgb.b = p;
        break;
    case 2:
        rgb.r = p; rgb.g = hsv.v; rgb.b = t;
        break;
    case 3:
        rgb.r = p; rgb.g = q; rgb.b = hsv.v;
        break;
    case 4:
        rgb.r = t; rgb.g = p; rgb.b = hsv.v;
        break;
    default:
        rgb.r = hsv.v; rgb.g = p; rgb.b = q;
        break;
    }

    return rgb;
}

HsvColor rgb2hsv(RgbColor rgb) {
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0) {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0) {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}

class PID {
  public:
    static const float kp;
    static const float ki;

    PID() {
        target = 0;
        value = 0;
        i_val = 0;
    }

    float update(float dt = 0.05) {
        float err = target - value;
        i_val += err * dt;
        value += err * kp + i_val * ki;
        return std::fmin(255.0, value);
    }

    void set_target(int val) {
        target = val;
    }
  private:
    float target;
    float value;
    float i_val;
};

const float PID::kp = 0.002;
const float PID::ki = 0.001;

const RgbColor BLACK = { 0, 0, 0 };

const std::array<const RgbColor, 5> CIVIL_PALETTE {{
        {  165, 113, 0 },
        {  165, 87, 0 },
        {  165, 75, 0 },
        {  170, 70, 0 },
        {  175, 67, 0 }
    }};

const std::array<const RgbColor, 5> NAUTICAL_PALETTE {{
        {  190, 67, 0 },
        {  190, 56, 0 },
        {  215, 56, 0 },
        {  215, 57, 2 },
        {  230, 57, 4 }
    }};

const std::array<const RgbColor, 5> ASTRONOMICAL_PALETTE {{
        {  240, 50, 6 },
        {  240, 40, 8 },
        {  240, 35, 10 },
        {  240, 30, 18 },
        {  230, 30, 30 }
    }};

const std::array<const RgbColor, 7> AURORA_PALETTE {{
        {  231, 14, 50 },
        {  240, 14, 172 },
        {  139, 2, 206 },
        {  67, 2, 206 },
        {  5, 221, 118 },
        {  5, 221, 87 },
        {  29, 255, 57 }
    }};

const int NUM_CONTROLLERS = NUM_LEDS * 3;

std::array<PID, NUM_CONTROLLERS> color_controllers;

void sendPacket(char* data, int len) {
    sockaddr_in addr = *((sockaddr_in*) addr_receiver);
    if (sendto(s, data, len, 0, (sockaddr*) &addr, sizeof(addr)) == -1) {
        printf("send to failed\n");
    }
}

void sendOSC(std::string addr, int value) {
    int len = tosc_writeMessage(
                  buffer, BUFFER_SIZE,
                  addr.c_str(), // the address
                  "i",   // the format; 'f':32-bit float, 's':ascii string, 'i':32-bit integer
                  value);
    sendPacket(buffer, len);
}

int to_byte(float val) {
    return std::min(255, static_cast<int>(std::round(val)));
}

void set_color(int i, RgbColor clr) {
    sendOSC(std::string("/") + std::to_string(i + 1) + std::string("/red"), to_byte(clr.r));
    sendOSC(std::string("/") + std::to_string(i + 1) + std::string("/green"), to_byte(clr.g));
    sendOSC(std::string("/") + std::to_string(i + 1) + std::string("/blue"), to_byte(clr.b));
}

int color_range = 110;

RgbColor generate_color() {
    HsvColor hsv;
    hsv.h = color_range + (rand() % 90);
    hsv.s = 255;
    hsv.v = 255;
    return hsv2rgb(hsv);
}

State get_state() {
    std::time_t now = time(0);
    tm *ltm = localtime(&now);
    // CHANGE TIMES
    if (ltm->tm_hour == 18) {
        if (ltm->tm_min >= 37) {
            return CIVIL;
        }
    } else if (ltm->tm_hour == 19) {
        return ltm->tm_min < 14 ? CIVIL : NAUTICAL;
    } else if (ltm->tm_hour == 20) {
        if (ltm->tm_min < 49)
            return ASTRONIMICAL;
        else
            return AURORA;
    } else if (ltm->tm_hour > 20 && ltm->tm_hour < 23) {
        return AURORA;
    }
    return OFF;
}

void change_color_thread_handle() {
    using namespace std::chrono_literals;
    std::vector<RgbColor> color_map;

    for (int i = 0; i < NUM_LEDS; ++i) {
        color_map.push_back(BLACK);
    }

    while (true) {
        std::rotate(color_map.rbegin(), color_map.rbegin() + 1, color_map.rend());        State state = get_state();

        std::time_t now = time(0);
        tm *ltm = localtime(&now);

        if (state == OFF) {
            color_map[0] = BLACK;
            printf("Time is %2d:%02d, State: OFF, R %d G %d B %d\n", ltm->tm_hour, ltm->tm_min, color_map[0].r, color_map[0].g, color_map[0].b);
        } else if (state == CIVIL) {
            int idx = rand() % CIVIL_PALETTE.size();
            color_map[0] = CIVIL_PALETTE[idx];
            printf("Time is %2d:%02d, State: CIVIL, R %d G %d B %d\n", ltm->tm_hour, ltm->tm_min, color_map[0].r, color_map[0].g, color_map[0].b);
        } else if (state == NAUTICAL) {
            int idx = rand() % NAUTICAL_PALETTE.size();
            color_map[0] = NAUTICAL_PALETTE[idx];
            printf("Time is %2d:%02d, State: NAUTICAL, R %d G %d B %d\n", ltm->tm_hour, ltm->tm_min, color_map[0].r, color_map[0].g, color_map[0].b);
        } else if (state == ASTRONIMICAL) {
            int idx = rand() % ASTRONOMICAL_PALETTE.size();
            color_map[0] = ASTRONOMICAL_PALETTE[idx];
            printf("Time is %2d:%02d, State: ASTRONIMICAL, R %d G %d B %d\n", ltm->tm_hour, ltm->tm_min, color_map[0].r, color_map[0].g, color_map[0].b);
        } else if (state == AURORA) {
            int idx = rand() % AURORA_PALETTE.size();
            color_map[0] = AURORA_PALETTE[idx];
            printf("Time is %2d:%02d, State: AURORA, R %d G %d B %d\n", ltm->tm_hour, ltm->tm_min, color_map[0].r, color_map[0].g, color_map[0].b);
        }

        for (int i = 0; i < NUM_LEDS; ++i) {
            color_controllers[i * 3].set_target(color_map[i].r);
            color_controllers[i * 3 + 1].set_target(color_map[i].g);
            color_controllers[i * 3 + 2].set_target(color_map[i].b);
        }
        std::this_thread::sleep_for(30s);
    }
}

int main() {
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_receiver = &addr;

    // create a UDP socket
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        printf("socket failed\n");
        return 0;
    }

    printf("Socket created.\n");

    for (int i = 0; i < NUM_CONTROLLERS; ++i) {
        color_controllers[i].set_target(255.0f);
    }

    std::thread change_color_thread(&change_color_thread_handle);

    while (true) {
        for (int i = 0; i < NUM_LEDS; ++i) {
            RgbColor clr;
            clr.r = color_controllers[i * 3].update();
            clr.g = color_controllers[i * 3 + 1].update();
            clr.b = color_controllers[i * 3 + 2].update();
            set_color(i, clr);
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(50ms);
    }

    change_color_thread.join();
    return 0;
}
