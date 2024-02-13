# STM32-bootloader
<br>
<p>In this repo there are some few various project for bootloader in <b>&quot;STM32F407ZGT6&quot;<b>.<p>
<br>
<p style="size: 12px">1. Bootloader Using SDCARD :</p>
<br>
<p>In this Project when bootloader program run so, it detect the sd card and install & run the application program</p>
<br>
<h2>2. Bootloader Using Uart (Serial) :<h2>
<p>In this Project when bootloader program run so, it receive the application program using serial communication and install & run it.</p>
<br>
<h2>3. Bootloader Using 2 Slot :<h2>
<br>
<p>In this Project we use teo different slot(sector) to store application program.<br>
In two Sector already two different application program store.<br>
1. Application-1 Test<br>
2. Application-2 Test<br>
when the bootloader program run then it check the user button's.</p><br>
<p>There are four case of user Button's :<br>
	1. If User button-1 is pressed then update the application 1 in define sector address.<br>
	2. If User button-2 is pressed then update the application 2 in define sector address.<br>
	3. If User button-3 is pressed then run the application 1 program.<br>
	4. If User button-4 is pressed then run the application 2 program.<br>
<br>