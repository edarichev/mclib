/**
 * MG90S servo motor
 */

#ifndef _MG90S_INCLUDED_
#define _MG90S_INCLUDED_

/**
 * MG90S с круговым вращением
 *
 * Установки:
 * TIM2->Clock source->Internal clock
 * Channel1->PWN generation CH1
 * 72МГц, длительность рабочего цикла 50Гц или 20мс.
 * 72'000'000/20=1'440'000
 * Prescaler = 1440000/1000=1440, 1440-1=1399;
 * ARR = 1000-1=999
 *
 * Ширина импульса регулирует частоту вращения.
 * 1мс - вращение по часовой стрелке
 * ~1.5мс - остановка
 * ~2мс - против часовой стрелки
 * Тогда 1мс => = ARR/20 = 1000/20=50, 1.5ms=1.5*50=75, 2ms=2*50=100
 * Это значение ставим в регистр htim2.Instance->CCR1 (или другого канала).
 * Отсюда следует, что вращение по часовой управляется значениями между 20 и 75,
 * против часовой - от 75 и больше.
 *
 * Определим параметры конкретного двигателя (посчитаем визуально и транспортиром
 * обороты за 10 секунд):
 * @code
    for (int i = 20; i <= 130; i += 1) {
        logger.writeLine("%d", i);
        htim2.Instance->CCR1 = i;
        HAL_Delay(10000);
        htim2.Instance->CCR1 = 0;
        HAL_Delay(30000);
    }
 *
 *
 * CRR = 35, 582,5 deg/sec.  1,62 об/с.
 * CRR = 73, 103.5 deg/sec.  0,29 об/с.
 * 74-76 - 0
 * 77, 107 deg/sec. 0,3 об/с.
 * 114, 550,5 deg/sec.  1,53 об/с.
 * Далее идёт не очень стабильная зона около 1.53...1.6 об/с
 * Зависимость от 35 до 73 почти линейная, от 76 до 114 не очень ровная, но почти линейная,
 * поэтому время и угол поворота задаём интерполяцией.
 *
 * Зная минимальные и максимальные обороты и значения CRR, можно определить
 * и задать частоту вращения и время вращения. На меньших оборотах момент силы M=F*R
 * будет меньше.
 *
 * Следует помнить, что нельзя задать скорость вращения меньше минимальной,
 * например, в этом случае не меньше 103 градусов по часовой стрелке.
 *
 */
class MG90S360
{
    uint32_t _cwMinValue; // ARR
    uint32_t _cwMaxValue; // ARR
    uint32_t _centerZeroValue; // ARR
    uint32_t _ccwMinValue; // ARR
    uint32_t _ccwMaxValue; // ARR
    float _cwMinSpeed; // градусов в секунду, по ч.с., соотв. _cwMaxValue
    float _cwMaxSpeed; // градусов в секунду, по ч.с., соотв. _cwMinValue
    float _ccwMinSpeed; // градусов в секунду, против ч.с.
    float _ccwMaxSpeed; // градусов в секунду, против ч.с.
public:
    MG90S360(uint32_t cwMinValue, uint32_t cwMaxValue,
            uint32_t centerZeroValue,
            uint32_t ccwMinValue, uint32_t ccwMaxValue,
            float cwMinSpeed, float cwMaxSpeed,
            float ccwMinSpeed, float ccwMaxSpeed)
    : _cwMinValue(cwMinValue), _cwMaxValue(cwMaxValue),
      _centerZeroValue(centerZeroValue),
      _ccwMinValue(ccwMinValue), _ccwMaxValue(ccwMaxValue),
      _cwMinSpeed(cwMinSpeed), _cwMaxSpeed(cwMaxSpeed),
      _ccwMinSpeed(ccwMinSpeed), _ccwMaxSpeed(ccwMaxSpeed)
    {

    }

    /**
     * Определяет время вращения по ч.с. с заданной скоростью
     *
     * @param ccrValue значение для установки в CCR регистр
     */
    void getRotateTimeCW(float degreesPerSecond, float degreesToRotate,
            /*out*/ uint32_t timeMsToRotate,
            /*out*/ uint32_t &ccrValue)
    {
        uint32_t delta = _cwMaxValue - _cwMinValue;
        //  w, об/с
        float deltaW = _cwMaxSpeed - _cwMinSpeed;
        float curSpeed = degreesPerSecond < _cwMinSpeed ? _cwMinSpeed : degreesPerSecond; // но не ниже минимальной
        if (curSpeed > _cwMaxSpeed)
            curSpeed = _cwMaxSpeed;
        float partSpeed = (curSpeed - _cwMinSpeed) / deltaW;
        ccrValue = _cwMaxValue - (1.0 * delta * partSpeed);
        timeMsToRotate = (uint32_t)(1000.0 * degreesToRotate / curSpeed); // ms
    }

    /**
     * Определяет время вращения против ч.с. с заданной скоростью
     *
     * @param ccrValue значение для установки в CCR регистр
     */
    void getRotateTimeCCW(float degreesPerSecond, float degreesToRotate,
            /*out*/ uint32_t &timeMsToRotate,
            /*out*/ uint32_t &ccrValue)
    {
        uint32_t delta = _ccwMaxValue - _ccwMinValue;
        //  w, об/с
        float deltaW = _ccwMaxSpeed - _ccwMinSpeed;
        float curSpeed = degreesPerSecond < _cwMinSpeed ? _cwMinSpeed : degreesPerSecond; // но не ниже минимальной
        if (curSpeed > _cwMaxSpeed)
            curSpeed = _cwMaxSpeed;
        float partSpeed = (curSpeed - _ccwMinSpeed) / deltaW;
        ccrValue = _ccwMinValue + (1.0 * delta * partSpeed);
        timeMsToRotate = (uint32_t)(1000.0 * degreesToRotate / curSpeed); // ms
    }
};

#endif // _MG90S_INCLUDED_ 
