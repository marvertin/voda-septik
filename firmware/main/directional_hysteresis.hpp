#pragma once

class DirectionalHysteresis {
public:
    explicit DirectionalHysteresis(float hysteresis)
        : hysteresis_(hysteresis), value_(0.0f), direction_(0), initialized_(false)
    {
    }

    float process(float input)
    {
        if (!initialized_) {
            value_ = input;
            initialized_ = true;
            return value_;
        }

        if (hysteresis_ <= 0.0f) {
            value_ = input;
            direction_ = 0;
            return value_;
        }

        const float delta = input - value_;
        if (delta == 0.0f) {
            return value_;
        }

        if (delta > 0.0f) {
            if (direction_ < 0 && delta < hysteresis_) {
                return value_;
            }
            direction_ = 1;
            value_ = input;
            return value_;
        }

        if (direction_ > 0 && delta > -hysteresis_) {
            return value_;
        }
        direction_ = -1;
        value_ = input;
        return value_;
    }

    int direction() const
    {
        return direction_;
    }

private:
    float hysteresis_;
    float value_;
    int direction_; // -1 = klesani, 0 = nezavazne, 1 = stoupani
    bool initialized_;
};
