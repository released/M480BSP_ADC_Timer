# M480BSP_ADC_Timer
 M480BSP_ADC_Timer

udpate @ 2024/07/02

1. use ADC channel 11 (PB.11) to measure ADC , with regular trigger and timer tirgger

2. collect ADC vaule into array and sorting , drop first 2 and last 2 data then do average , refer to Sort_tab

3. enable define : ENABLE_ADC_TRIG_BY_TIMER , to enable trigger ADC by timer3 

4. below is trigger ADC by regulator IRQ , when NOT enable : ENABLE_ADC_TRIG_BY_TIMER


![image](https://github.com/released/M480BSP_ADC_Timer/main/adc_trig_by_cvt.jpg)


![image](https://github.com/released/M480BSP_ADC_Timer/main/adc_trig_by_cvt_3V3.jpg)


5. below is trigger ADC by timer3 , set compare vaule to 36 


![image](https://github.com/released/M480BSP_ADC_Timer/main/adc_trig_by_timer_36.jpg)


![image](https://github.com/released/M480BSP_ADC_Timer/main/adc_trig_by_timer_36_3V3.jpg)



6. below is trigger ADC by timer3 , set compare vaule to 5 


![image](https://github.com/released/M480BSP_ADC_Timer/main/adc_trig_by_timer_5.jpg)


![image](https://github.com/released/M480BSP_ADC_Timer/main/adc_trig_by_timer_5_3V3.jpg)

