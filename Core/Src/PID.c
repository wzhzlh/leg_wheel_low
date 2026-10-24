#include "PID.h"

/**
 * @brief 标准位置式PID
 * @param[in] current  实际值
 * @param[in] expected 期望值
 * @param[in] data     PID参数
 */
void PID_Control(float current, float expected, PID *data)
{
    data->error_last = data->error_now;
    data->error_now  = expected - current;
    data->error_inter += data->error_now;

    limit(data->error_inter, data->limit, -data->limit);

    data->pid_out = data->Kp * data->error_now
                  + data->Ki * data->error_inter
                  + data->Kd * (data->error_now - data->error_last);

    limit(data->pid_out, data->output_limit, -data->output_limit);
}

/**
 * @brief 增量式PID
 * @param[in] current   实际值
 * @param[in] expect    期望值
 * @param[in] parameter PID参数
 * @return PID增量
 */
float PID_Increment(float current, float expect, PID_ADD *parameter)
{
    parameter->error_last = parameter->error_next;
    parameter->error_next = parameter->error_now;
    parameter->error_now  = expect - current;

    parameter->increament = parameter->Kp * (parameter->error_now - parameter->error_next)
                          + parameter->Ki * parameter->error_now
                          + parameter->Kd * (parameter->error_now - 2.0f * parameter->error_next + parameter->error_last);

    return parameter->increament;
}
