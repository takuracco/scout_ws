class PID{
    private:
        float t;
        float kp;
        float ki;
        float kd;
        float past_e;
        float integral;

    public:
        PID(float _t):t(_t), kp(0.0f), ki(0.0f), kd(0.0f), past_e(0.0f), integral(0.0f){}

        void reset(){
            past_e = 0.0f;
            integral = 0.0f;
        }

        void SetParam(float _kp, float _ki, float _kd){
            kp = _kp;
            ki = _ki;
            kd = _kd;
        }

        float culcurate(float target, float measure){
            float e = target - measure;
            float p = kp * e;
            integral += ki * (e + past_e) / 2.f * t;
            float d = kd * (e - past_e) / t;
            float u = p + integral + d;
            past_e = e;
            return u;
        }
};