<h1>STM32H7 ADC Multi Channel Multi Conversion TriggerTimer with Oversample and data transmit with DMA</h1>
<p>&nbsp;</p>
<p>Tested with STM32H750 in HAL & LL library, should be able ported to others.<br>
This example will convert several channels in burst mode from HW trigger: timer peripheral.<br>
Oversample ratio set to max 1024 and leftshift 10bits to average 1024 sample.<br>
DMA is configured to transfer conversion data in an array, in circular mode.<br>
Timer is configured in time base and to generate TRGO events.</p>

<p>Disable Discontinuous conversion for burst conversion<br>
Disable continuous conversion for single conversion<br>
End of conversion selection set to "End of sequence Conversion"</p>

<img src="Images/Setting.jpg" width="30%" height="30%">


<p>&nbsp;</p>
<p>To calculate minimum conversion time:<br>
ADCFreq = 37.5 / 2 = 18.75MHz<br>
Minimum Tconv = 1.5 + 0.5 + (16/2) = 10 cycle<br>
Sample Time = 10 cycle / 18.75 MHz = 0.533 us<br>
According the Datasheet, Vref need at least 4.3us, so minimum Vref sampling time must 8.5 cycles.</p>
<img src="Images/Vref.jpg" width="30%" height="30%">

<p>&nbsp;</p>
<p>The Rank Setting:</p>
<img src="Images/Rank.jpg" width="30%" height="30%">

<p>&nbsp;</p>
<p>Using basic timer TIM6 and set it to 10kHz(10000 cycle/second),<br>
10000 counter to trigger event at 1 second interval.</p>

<img src="Images/Timer.jpg" width="30%" height="30%">

<p>&nbsp;</p>
<p>ADC value is convert to voltage with build in macro,<br>
ADC_INP0 --> GND<br>
ADC_INP1 --> VCC<br>
The Output in Live Expression:</p>

<img src="Images/Live.jpg" width="30%" height="30%">

<p>&nbsp;</p>
<p>Check the logic pin to see if the ADC is converted with 1sec interval.</p>

<img src="Images/Logic_Time.jpg" width="50%" height="50%">

<p>&nbsp;</p>
<p>Comment/Suggestion are highly welcome!</p>