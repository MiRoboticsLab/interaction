#include "i420_creator.h"
#include <cassert>
#include <future>
#include <chrono>
#include <iostream>

I420Creator::~I420Creator()
{
    running_ = false;
    std::cout << "destructing i420 creator" << std::endl;
    bool joinable = thread_.joinable();
    std::cout << "joinable = " << joinable << std::endl;
    if(joinable){
        thread_.join();
    }
    std::cout << "joined" << std::endl;
}

void I420Creator::run(int fps)
{
    if(running_ || fps == 0) {
        assert(false);
        return;
    }
    running_ = true;
    std::promise<bool> promise;
    auto future = promise.get_future();
    thread_ = std::thread([this, fps, &promise]()
    {
        promise.set_value(true);
        while(running_) {
            static int i = 0;
            auto duration_ms = 1000 / fps;
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1ms * duration_ms);
            auto nowTime = std::chrono::system_clock::now();
            auto nanoSeconds = std::chrono::duration_cast<std::chrono::nanoseconds>(nowTime.time_since_epoch()).count();
            if(on_frame_) {
                on_frame_(process()->data(), nanoSeconds, i);
            }
            ++i;
        }
    });
    future.wait();
}

uint8_t limit(int& v, int min, int max)
{
    v = std::min(max, v);
    v = std::max(min, v);
    return static_cast<uint8_t>(v);
}

void rgb_to_i420(const uint8_t* rgb, uint8_t* yuv, size_t size)
{
    assert(size >= 3);
    auto r = rgb[0];
    auto g = rgb[1];
    auto b = rgb[2];

    int y = ((66  * r + 129 * g + 25  * b + 128) >> 8) + 16;
    int u = ((-38 * r - 74  * g + 112 * b + 128) >> 8) + 128;
    int v = ((112 * r - 94  * g - 18  * b + 128) >> 8) + 128;

    yuv[0] = limit(y, 0, 255);
    yuv[1] = limit(u, 0, 255);
    yuv[2] = limit(v, 0, 255);
}

I420Creator::I420Frame I420Creator::process()
{
    static int counts = 0;
    counts++;
    const uint8_t colors[6][3] =
    {                   //RGB
        {255, 0, 0},    //red
        {255, 165, 0},  //orange
        {255, 255, 0},  //yellow
        {0, 255, 0},    //Green
        {0, 0, 255},    //Blue
        {160,32,240}    //purple
    };
    static int i = 0;
    i = (i++) % 6;
    auto frame = std::make_shared<std::vector<uint8_t>>();
    frame->resize(static_cast<size_t>(w_ * h_ * 3 / 2));
    uint8_t* buffer_y = frame->data();
    uint8_t* buffer_u = frame->data() + w_*h_;
    uint8_t* buffer_v = buffer_u + w_*h_ / 4;
    for(size_t i = 0 ;i < h_; i++)
    {
        for(size_t j = 0; j < w_; j++)
        {
            const auto& rgb = colors[j % 6];
            uint8_t yuv[3] = {0};
            rgb_to_i420(rgb, yuv, 3);
            *(buffer_y++) = yuv[0];
            bool set_uv = false;
            if(j % 2 == 0 && i %2 == 0)
            {
                *(buffer_u++) = yuv[1];
                *(buffer_v++) = yuv[2];
                set_uv = true;
            }
            if (i >= h_ / 4 * (counts / 2 % 4) && i <= h_ / 4 * ((counts / 2 % 4) + 1)) {
                const auto& purple = colors[5];
                uint8_t yuv_p[3] = {0};
                rgb_to_i420(purple, yuv_p, 3);
                *(buffer_y - 1) = yuv_p[0];
                if (set_uv) {
                    *(buffer_u - 1) = yuv_p[1];
                    *(buffer_v - 1) = yuv_p[2];
                } else {
                    *buffer_u = yuv_p[1];
                    *buffer_v = yuv_p[2];
                }
            }
        }
    }
    return frame;
}
