EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "MotionControlled Bicicle Backlight"
Date "2020-09-28"
Rev "0.1"
Comp "jumble"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MoCoBiBa_no_charger-rescue:MPU-6500-custom-MoCoBiBa_no_charger-rescue U5
U 1 1 5F7280D1
P 9650 2000
F 0 "U5" H 10294 2046 50  0000 L CNN
F 1 "MPU-6500" H 10294 1955 50  0000 L CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_3x3mm_P0.4mm" H 9650 2000 50  0001 C CNN
F 3 "" H 9650 2000 50  0001 C CNN
	1    9650 2000
	1    0    0    -1  
$EndComp
Text Notes 1000 5000 0    50   ~ 0
Mounting Hole\n
$Comp
L MoCoBiBa_no_charger-rescue:LM3671MF-2.8-lm3671mf-2.8-MoCoBiBa_no_charger-rescue U4
U 1 1 5F7485F1
P 9000 5700
F 0 "U4" H 9000 6025 50  0000 C CNN
F 1 "LM3671MF-3.3" H 9000 5934 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TSOT-23-5" H 9050 5450 50  0001 L CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/ADP2108.pdf" H 8750 5350 50  0001 C CNN
	1    9000 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5F74896B
P 9000 6050
F 0 "#PWR021" H 9000 5800 50  0001 C CNN
F 1 "GND" H 9005 5877 50  0000 C CNN
F 2 "" H 9000 6050 50  0001 C CNN
F 3 "" H 9000 6050 50  0001 C CNN
	1    9000 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 6050 9000 6000
$Comp
L Device:L L1
U 1 1 5F748E03
P 9500 5600
F 0 "L1" V 9690 5600 50  0000 C CNN
F 1 "2.2u" V 9599 5600 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9500 5600 50  0001 C CNN
F 3 "~" H 9500 5600 50  0001 C CNN
	1    9500 5600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9300 5600 9350 5600
Wire Wire Line
	9300 5800 9700 5800
Wire Wire Line
	9700 5800 9700 5600
Wire Wire Line
	9700 5600 9650 5600
$Comp
L Device:C_Small C5
U 1 1 5F74A3BA
P 9900 5700
F 0 "C5" H 9992 5746 50  0000 L CNN
F 1 "10u" H 9992 5655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9900 5700 50  0001 C CNN
F 3 "~" H 9900 5700 50  0001 C CNN
	1    9900 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 5600 9900 5600
Connection ~ 9700 5600
$Comp
L power:GND #PWR028
U 1 1 5F74A64F
P 9900 5950
F 0 "#PWR028" H 9900 5700 50  0001 C CNN
F 1 "GND" H 9905 5777 50  0000 C CNN
F 2 "" H 9900 5950 50  0001 C CNN
F 3 "" H 9900 5950 50  0001 C CNN
	1    9900 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 5950 9900 5800
Wire Wire Line
	10200 5600 9900 5600
Connection ~ 9900 5600
$Comp
L Device:R R10
U 1 1 5F74AE3D
P 8400 5700
F 0 "R10" V 8600 5700 50  0000 C CNN
F 1 "100k" V 8500 5700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8330 5700 50  0001 C CNN
F 3 "~" H 8400 5700 50  0001 C CNN
	1    8400 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	8550 5700 8700 5700
Wire Wire Line
	8700 5600 8250 5600
Wire Wire Line
	8250 5600 8250 5700
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 5F7539AB
P 3550 3950
F 0 "J2" H 3658 4131 50  0000 C CNN
F 1 "Battery Connector" H 3658 4040 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 3550 3950 50  0001 C CNN
F 3 "~" H 3550 3950 50  0001 C CNN
	1    3550 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 3950 4100 3950
$Comp
L power:VCC #PWR019
U 1 1 5F7A1F04
P 8100 5600
F 0 "#PWR019" H 8100 5450 50  0001 C CNN
F 1 "VCC" H 8115 5773 50  0000 C CNN
F 2 "" H 8100 5600 50  0001 C CNN
F 3 "" H 8100 5600 50  0001 C CNN
	1    8100 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 5600 8250 5600
Connection ~ 8250 5600
NoConn ~ 10250 1950
NoConn ~ 10250 2050
NoConn ~ 10250 2150
NoConn ~ 10250 1850
NoConn ~ 9050 1750
NoConn ~ 9050 1850
NoConn ~ 9050 1950
NoConn ~ 9050 2050
NoConn ~ 9050 2150
NoConn ~ 9050 2250
NoConn ~ 9900 1400
NoConn ~ 9400 2600
NoConn ~ 9800 2600
NoConn ~ 9700 1400
$Comp
L power:GND #PWR031
U 1 1 5F7D0083
P 10750 1750
F 0 "#PWR031" H 10750 1500 50  0001 C CNN
F 1 "GND" V 10755 1622 50  0000 R CNN
F 2 "" H 10750 1750 50  0001 C CNN
F 3 "" H 10750 1750 50  0001 C CNN
	1    10750 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10750 1750 10250 1750
$Comp
L power:GND #PWR026
U 1 1 5F7D1BEE
P 9800 1050
F 0 "#PWR026" H 9800 800 50  0001 C CNN
F 1 "GND" H 9805 877 50  0000 C CNN
F 2 "" H 9800 1050 50  0001 C CNN
F 3 "" H 9800 1050 50  0001 C CNN
	1    9800 1050
	-1   0    0    1   
$EndComp
Wire Wire Line
	9800 1050 9800 1400
$Comp
L power:GND #PWR025
U 1 1 5F7D30AB
P 9800 3150
F 0 "#PWR025" H 9800 2900 50  0001 C CNN
F 1 "GND" H 9805 2977 50  0000 C CNN
F 2 "" H 9800 3150 50  0001 C CNN
F 3 "" H 9800 3150 50  0001 C CNN
	1    9800 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5F7D35A8
P 9500 3150
F 0 "#PWR023" H 9500 2900 50  0001 C CNN
F 1 "GND" H 9505 2977 50  0000 C CNN
F 2 "" H 9500 3150 50  0001 C CNN
F 3 "" H 9500 3150 50  0001 C CNN
	1    9500 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5F7D393F
P 9800 2850
F 0 "C4" H 9892 2896 50  0000 L CNN
F 1 "0.1u" H 9892 2805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9800 2850 50  0001 C CNN
F 3 "~" H 9800 2850 50  0001 C CNN
	1    9800 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5F7D3DC0
P 9500 3000
F 0 "C3" H 9300 2950 50  0000 L CNN
F 1 "10n" H 9250 2850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9500 3000 50  0001 C CNN
F 3 "~" H 9500 3000 50  0001 C CNN
	1    9500 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 2950 9800 3050
Wire Wire Line
	9500 2600 9500 2850
Wire Wire Line
	8900 2850 9500 2850
Connection ~ 9500 2850
Wire Wire Line
	9500 2850 9500 2900
Wire Wire Line
	10850 2250 10650 2250
$Comp
L Device:C_Small C6
U 1 1 5F7DA892
P 10650 2400
F 0 "C6" H 10742 2446 50  0000 L CNN
F 1 "0.1u" H 10742 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10650 2400 50  0001 C CNN
F 3 "~" H 10650 2400 50  0001 C CNN
	1    10650 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5F7DAAD3
P 10650 2600
F 0 "#PWR030" H 10650 2350 50  0001 C CNN
F 1 "GND" H 10655 2427 50  0000 C CNN
F 2 "" H 10650 2600 50  0001 C CNN
F 3 "" H 10650 2600 50  0001 C CNN
	1    10650 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 2600 10650 2500
Wire Wire Line
	10650 2300 10650 2250
Connection ~ 10650 2250
Wire Wire Line
	10650 2250 10250 2250
Wire Wire Line
	9600 1000 9600 1400
Text GLabel 8950 1000 0    50   Input ~ 0
SCL
Text GLabel 8950 1150 0    50   Input ~ 0
SDA
Wire Wire Line
	8950 1150 9050 1150
Wire Wire Line
	9400 1150 9400 1400
Wire Wire Line
	8950 1000 9100 1000
Wire Wire Line
	9500 1000 9500 1400
Connection ~ 9800 3050
Wire Wire Line
	9800 3050 9800 3150
Text GLabel 10050 2650 2    50   Input ~ 0
Accel_Int
Wire Wire Line
	9900 2600 9900 2650
Wire Wire Line
	9900 2650 10050 2650
Text GLabel 3700 1700 2    50   Input ~ 0
SDA
Wire Wire Line
	3700 1700 3650 1700
$Comp
L Connector:AVR-ISP-6 J3
U 1 1 5F7F1F43
P 6500 4050
F 0 "J3" H 6171 4146 50  0000 R CNN
F 1 "AVR-ISP-6" H 6171 4055 50  0000 R CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x03_P2.54mm_Vertical" V 6250 4100 50  0001 C CNN
F 3 " ~" H 5225 3500 50  0001 C CNN
	1    6500 4050
	1    0    0    -1  
$EndComp
Text GLabel 3700 1600 2    50   Input ~ 0
MISO_ISP
Wire Wire Line
	3700 1600 3100 1600
Text GLabel 3700 1800 2    50   Input ~ 0
MOSI_ISP
Wire Wire Line
	3650 1800 3650 1700
Connection ~ 3650 1700
Wire Wire Line
	3650 1700 3100 1700
Text GLabel 3700 2300 2    50   Input ~ 0
nRES_ISP
Wire Wire Line
	3700 2300 3100 2300
Text GLabel 7200 4050 2    50   Input ~ 0
SCK_ISP
Wire Wire Line
	7200 4050 6900 4050
Text GLabel 7200 3850 2    50   Input ~ 0
MISO_ISP
Wire Wire Line
	7200 3850 6900 3850
Text GLabel 7200 3950 2    50   Input ~ 0
MOSI_ISP
Wire Wire Line
	7200 3950 6900 3950
Text GLabel 7200 4150 2    50   Input ~ 0
nRES_ISP
Wire Wire Line
	7200 4150 6900 4150
$Comp
L power:GND #PWR06
U 1 1 5F809811
P 2500 2750
F 0 "#PWR06" H 2500 2500 50  0001 C CNN
F 1 "GND" H 2505 2577 50  0000 C CNN
F 2 "" H 2500 2750 50  0001 C CNN
F 3 "" H 2500 2750 50  0001 C CNN
	1    2500 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 2750 2500 2600
$Comp
L power:GND #PWR016
U 1 1 5F80C429
P 6400 4550
F 0 "#PWR016" H 6400 4300 50  0001 C CNN
F 1 "GND" H 6405 4377 50  0000 C CNN
F 2 "" H 6400 4550 50  0001 C CNN
F 3 "" H 6400 4550 50  0001 C CNN
	1    6400 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4550 6400 4450
$Comp
L Device:LED_Small D4
U 1 1 5F7509F3
P 7050 1300
F 0 "D4" V 7096 1230 50  0000 R CNN
F 1 "LED 8-4500" V 7005 1230 50  0000 R CNN
F 2 "LED_SMD:LED_PLCC-2" V 7050 1300 50  0001 C CNN
F 3 "https://cdn-reichelt.de/documents/datenblatt/A500/LED8-4500RT%23KIN.pdf" V 7050 1300 50  0001 C CNN
	1    7050 1300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small D3
U 1 1 5F76B89E
P 6750 1300
F 0 "D3" V 6750 1230 50  0000 R CNN
F 1 "LED 8-4500" V 6705 1230 50  0001 R CNN
F 2 "LED_SMD:LED_PLCC-2" V 6750 1300 50  0001 C CNN
F 3 "https://cdn-reichelt.de/documents/datenblatt/A500/LED8-4500RT%23KIN.pdf" V 6750 1300 50  0001 C CNN
	1    6750 1300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small D2
U 1 1 5F76C2B4
P 6450 1300
F 0 "D2" V 6450 1230 50  0000 R CNN
F 1 "LED 8-4500" V 6405 1230 50  0001 R CNN
F 2 "LED_SMD:LED_PLCC-2" V 6450 1300 50  0001 C CNN
F 3 "https://cdn-reichelt.de/documents/datenblatt/A500/LED8-4500RT%23KIN.pdf" V 6450 1300 50  0001 C CNN
	1    6450 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7050 1200 7050 1100
Wire Wire Line
	7050 1100 6750 1100
Wire Wire Line
	6750 1100 6750 1200
Wire Wire Line
	6450 1200 6450 1100
Wire Wire Line
	6450 1100 6750 1100
Connection ~ 6750 1100
Wire Wire Line
	6750 1050 6750 1100
Wire Wire Line
	7050 1400 7050 1500
Wire Wire Line
	7050 1500 6750 1500
Wire Wire Line
	6750 1400 6750 1500
Connection ~ 6750 1500
Wire Wire Line
	6450 1400 6450 1500
Wire Wire Line
	6450 1500 6750 1500
$Comp
L power:GND #PWR018
U 1 1 5F780B86
P 6750 2400
F 0 "#PWR018" H 6750 2150 50  0001 C CNN
F 1 "GND" H 6755 2227 50  0000 C CNN
F 2 "" H 6750 2400 50  0001 C CNN
F 3 "" H 6750 2400 50  0001 C CNN
	1    6750 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2400 6750 2200
$Comp
L Device:R R8
U 1 1 5F78433B
P 6350 2250
F 0 "R8" H 6420 2296 50  0000 L CNN
F 1 "10k" H 6420 2205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6280 2250 50  0001 C CNN
F 3 "~" H 6350 2250 50  0001 C CNN
	1    6350 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 2400 6750 2400
Connection ~ 6750 2400
$Comp
L Device:R R7
U 1 1 5F78B496
P 6100 2100
F 0 "R7" V 6307 2100 50  0000 C CNN
F 1 "1k" V 6216 2100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6030 2100 50  0001 C CNN
F 3 "~" H 6100 2100 50  0001 C CNN
	1    6100 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3100 2200 3700 2200
Text Notes 6000 2800 0    50   ~ 0
pulsed signal\nno resistor used to increase efficency, \nPower consumption controlled by duty cycle: \nmax dutycycle: 70%
$Comp
L Device:R R9
U 1 1 5F7C6565
P 6750 1650
F 0 "R9" H 6820 1696 50  0000 L CNN
F 1 "5" H 6820 1605 50  0000 L CNN
F 2 "Resistor_SMD:R_0815_2038Metric_Pad1.53x4.00mm_HandSolder" V 6680 1650 50  0001 C CNN
F 3 "~" H 6750 1650 50  0001 C CNN
	1    6750 1650
	1    0    0    -1  
$EndComp
Text Notes 8000 4900 0    50   ~ 0
Buck Converter 3.3V, 600mA
Text GLabel 3700 2200 2    50   Input ~ 0
LED_PWM
Text GLabel 5950 2100 0    50   Input ~ 0
LED_PWM
Wire Notes Line
	950  5050 3450 5050
Wire Notes Line
	3450 5050 3450 7100
Wire Notes Line
	950  7100 950  5050
Wire Notes Line
	7800 4950 7800 6450
Wire Notes Line
	7800 6450 10500 6450
Wire Notes Line
	10500 6450 10500 4950
Wire Notes Line
	10500 4950 7800 4950
Wire Notes Line
	5500 3300 5500 4850
Wire Notes Line
	5500 4850 7850 4850
Wire Notes Line
	7850 4850 7850 3300
Wire Notes Line
	7850 3300 5500 3300
Wire Notes Line
	7850 650  7850 3100
Wire Notes Line
	5500 650  5500 3100
Wire Notes Line
	8350 750  8350 3550
Wire Notes Line
	8350 3550 11150 3550
Wire Notes Line
	11150 3550 11150 750 
Wire Notes Line
	11150 750  8350 750 
Wire Notes Line
	5500 3100 7850 3100
Wire Notes Line
	5500 650  7850 650 
Wire Notes Line
	3050 3250 3050 4750
Wire Notes Line
	3050 4750 4550 4750
Wire Notes Line
	4550 4750 4550 3250
Wire Notes Line
	3050 3250 4550 3250
Wire Notes Line
	1200 550  1200 3000
Wire Notes Line
	1200 3000 4500 3000
Wire Notes Line
	1200 550  4500 550 
Text Notes 3200 3200 0    50   ~ 0
Battery Connector
Text Notes 1300 650  0    50   ~ 0
Main Processing Unit
Text Notes 5650 600  0    50   ~ 0
LED Driver Circuit
Text Notes 8550 700  0    50   ~ 0
IMU (MPU-6500)
Text Notes 5600 3250 0    50   ~ 0
Programming Interface
Text GLabel 10000 4550 2    50   Input ~ 0
SCL
Text GLabel 9550 4550 2    50   Input ~ 0
SDA
$Comp
L Device:R R12
U 1 1 5F91C07D
P 9900 4300
F 0 "R12" H 9970 4346 50  0000 L CNN
F 1 "R" H 9970 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9830 4300 50  0001 C CNN
F 3 "~" H 9900 4300 50  0001 C CNN
	1    9900 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 5F91C50E
P 9450 4300
F 0 "R11" H 9520 4346 50  0000 L CNN
F 1 "R" H 9520 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9380 4300 50  0001 C CNN
F 3 "~" H 9450 4300 50  0001 C CNN
	1    9450 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 4450 9450 4550
Wire Wire Line
	9450 4550 9550 4550
Wire Wire Line
	9900 4450 9900 4550
Wire Wire Line
	9900 4550 10000 4550
Wire Wire Line
	9900 4100 9900 4150
Wire Wire Line
	9450 4100 9450 4150
Wire Notes Line
	9200 3700 10400 3700
Wire Notes Line
	10400 3700 10400 4700
Wire Notes Line
	10400 4700 9200 4700
Wire Notes Line
	9200 3700 9200 4700
Text Notes 9300 3650 0    50   ~ 0
I2C Pullup Resistor
Text Notes 4300 750  0    50   ~ 0
PA2 Sinks current, so the \nvoltage divider can be turned off\n when not needed
Text GLabel 3200 1100 2    50   Input ~ 0
Accel_Int
NoConn ~ 3100 1800
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5F9CD702
P 4100 3950
F 0 "#FLG0101" H 4100 4025 50  0001 C CNN
F 1 "PWR_FLAG" H 4100 4123 50  0000 C CNN
F 2 "" H 4100 3950 50  0001 C CNN
F 3 "~" H 4100 3950 50  0001 C CNN
	1    4100 3950
	1    0    0    -1  
$EndComp
Connection ~ 4100 3950
Wire Wire Line
	4100 3950 3750 3950
Wire Wire Line
	9500 3100 9500 3150
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5FA0C18D
P 9900 5600
F 0 "#FLG0103" H 9900 5675 50  0001 C CNN
F 1 "PWR_FLAG" H 9900 5773 50  0000 C CNN
F 2 "" H 9900 5600 50  0001 C CNN
F 3 "~" H 9900 5600 50  0001 C CNN
	1    9900 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 2750 9700 2750
Wire Wire Line
	9700 2750 9700 2600
$Comp
L Device:R R13
U 1 1 5FA26563
P 9600 2850
F 0 "R13" H 9670 2896 50  0000 L CNN
F 1 "1k" H 9670 2805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9530 2850 50  0001 C CNN
F 3 "~" H 9600 2850 50  0001 C CNN
	1    9600 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 2700 9600 2600
Wire Wire Line
	9600 3000 9600 3050
Wire Wire Line
	9600 3050 9800 3050
Wire Wire Line
	3750 4050 4100 4050
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5FA3E7D6
P 4100 4050
F 0 "#FLG0104" H 4100 4125 50  0001 C CNN
F 1 "PWR_FLAG" H 4100 4223 50  0000 C CNN
F 2 "" H 4100 4050 50  0001 C CNN
F 3 "~" H 4100 4050 50  0001 C CNN
	1    4100 4050
	-1   0    0    1   
$EndComp
Connection ~ 4100 4050
Wire Wire Line
	4100 4050 4150 4050
Wire Wire Line
	3200 1100 3100 1100
$Comp
L power:VCC #PWR02
U 1 1 5F8B365C
P 4150 3950
F 0 "#PWR02" H 4150 3800 50  0001 C CNN
F 1 "VCC" V 4165 4078 50  0000 L CNN
F 2 "" H 4150 3950 50  0001 C CNN
F 3 "" H 4150 3950 50  0001 C CNN
	1    4150 3950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5F8B3D02
P 4150 4050
F 0 "#PWR03" H 4150 3800 50  0001 C CNN
F 1 "GND" V 4155 3922 50  0000 R CNN
F 2 "" H 4150 4050 50  0001 C CNN
F 3 "" H 4150 4050 50  0001 C CNN
	1    4150 4050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F97847D
P 3950 1150
F 0 "R5" V 4157 1150 50  0000 C CNN
F 1 "100k" V 4066 1150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3880 1150 50  0001 C CNN
F 3 "~" H 3950 1150 50  0001 C CNN
	1    3950 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5F963E08
P 3950 850
F 0 "R4" V 4157 850 50  0000 C CNN
F 1 "100k" V 4066 850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3880 850 50  0001 C CNN
F 3 "~" H 3950 850 50  0001 C CNN
	1    3950 850 
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR01
U 1 1 5F8BA71F
P 3950 700
F 0 "#PWR01" H 3950 550 50  0001 C CNN
F 1 "VCC" H 3965 873 50  0000 C CNN
F 2 "" H 3950 700 50  0001 C CNN
F 3 "" H 3950 700 50  0001 C CNN
	1    3950 700 
	1    0    0    -1  
$EndComp
$Comp
L dk_Transistors-FETs-MOSFETs-Single:IRLML2502TRPBF Q1
U 1 1 5F8C610E
P 6750 2000
F 0 "Q1" H 6858 2053 60  0000 L CNN
F 1 "IRLML2502TRPBF" H 6858 1947 60  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 6950 2200 60  0001 L CNN
F 3 "https://www.infineon.com/dgdl/irlml2502pbf.pdf?fileId=5546d462533600a401535668048e2606" H 6950 2300 60  0001 L CNN
F 4 "IRLML2502TRPBFCT-ND" H 6950 2400 60  0001 L CNN "Digi-Key_PN"
F 5 "IRLML2502TRPBF" H 6950 2500 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 6950 2600 60  0001 L CNN "Category"
F 7 "Transistors - FETs, MOSFETs - Single" H 6950 2700 60  0001 L CNN "Family"
F 8 "https://www.infineon.com/dgdl/irlml2502pbf.pdf?fileId=5546d462533600a401535668048e2606" H 6950 2800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/infineon-technologies/IRLML2502TRPBF/IRLML2502TRPBFCT-ND/812502" H 6950 2900 60  0001 L CNN "DK_Detail_Page"
F 10 "MOSFET N-CH 20V 4.2A SOT-23" H 6950 3000 60  0001 L CNN "Description"
F 11 "Infineon Technologies" H 6950 3100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 6950 3200 60  0001 L CNN "Status"
	1    6750 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2100 6350 2100
Connection ~ 6350 2100
Wire Wire Line
	6350 2100 6450 2100
$Comp
L Device:LED_Small D6
U 1 1 5F8DBD5F
P 6150 1300
F 0 "D6" V 6150 1230 50  0000 R CNN
F 1 "LED 8-4500" V 6105 1230 50  0001 R CNN
F 2 "LED_SMD:LED_PLCC-2" V 6150 1300 50  0001 C CNN
F 3 "https://cdn-reichelt.de/documents/datenblatt/A500/LED8-4500RT%23KIN.pdf" V 6150 1300 50  0001 C CNN
	1    6150 1300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small D5
U 1 1 5F8DC12F
P 5850 1300
F 0 "D5" V 5850 1230 50  0000 R CNN
F 1 "LED 8-4500" V 5805 1230 50  0001 R CNN
F 2 "LED_SMD:LED_PLCC-2" V 5850 1300 50  0001 C CNN
F 3 "https://cdn-reichelt.de/documents/datenblatt/A500/LED8-4500RT%23KIN.pdf" V 5850 1300 50  0001 C CNN
	1    5850 1300
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small D1
U 1 1 5F8DC40D
P 5600 1300
F 0 "D1" V 5600 1230 50  0000 R CNN
F 1 "LED 8-4500" V 5555 1230 50  0001 R CNN
F 2 "LED_SMD:LED_PLCC-2" V 5600 1300 50  0001 C CNN
F 3 "https://cdn-reichelt.de/documents/datenblatt/A500/LED8-4500RT%23KIN.pdf" V 5600 1300 50  0001 C CNN
	1    5600 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6450 1100 6150 1100
Wire Wire Line
	5600 1100 5600 1200
Connection ~ 6450 1100
Wire Wire Line
	5850 1200 5850 1100
Connection ~ 5850 1100
Wire Wire Line
	5850 1100 5600 1100
Wire Wire Line
	6150 1200 6150 1100
Connection ~ 6150 1100
Wire Wire Line
	6150 1100 5850 1100
Wire Wire Line
	5600 1400 5600 1500
Wire Wire Line
	5600 1500 5850 1500
Connection ~ 6450 1500
Wire Wire Line
	6150 1400 6150 1500
Connection ~ 6150 1500
Wire Wire Line
	6150 1500 6450 1500
Wire Wire Line
	5850 1400 5850 1500
Connection ~ 5850 1500
Wire Wire Line
	5850 1500 6150 1500
Wire Wire Line
	2500 750  2500 800 
Connection ~ 2500 800 
$Comp
L MCU_Microchip_ATtiny:ATtiny84-20PU U2
U 1 1 5F729DB3
P 2500 1700
F 0 "U2" H 1971 1746 50  0000 R CNN
F 1 "ATtiny84-20PU" H 1971 1655 50  0000 R CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 2500 1700 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc8006.pdf" H 2500 1700 50  0001 C CNN
	1    2500 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5F91F0F1
P 1600 950
F 0 "C1" H 1692 996 50  0000 L CNN
F 1 "1u" H 1692 905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1600 950 50  0001 C CNN
F 3 "~" H 1600 950 50  0001 C CNN
	1    1600 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 850  1600 800 
Wire Wire Line
	1600 800  2500 800 
$Comp
L power:GND #PWR04
U 1 1 5F922FD3
P 1600 1100
F 0 "#PWR04" H 1600 850 50  0001 C CNN
F 1 "GND" H 1605 927 50  0000 C CNN
F 2 "" H 1600 1100 50  0001 C CNN
F 3 "" H 1600 1100 50  0001 C CNN
	1    1600 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1100 1600 1050
$Comp
L power:+3.3V #PWR0101
U 1 1 5F8F1819
P 6750 1050
F 0 "#PWR0101" H 6750 900 50  0001 C CNN
F 1 "+3.3V" H 6765 1223 50  0000 C CNN
F 2 "" H 6750 1050 50  0001 C CNN
F 3 "" H 6750 1050 50  0001 C CNN
	1    6750 1050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5F8F1ABD
P 10200 5600
F 0 "#PWR0102" H 10200 5450 50  0001 C CNN
F 1 "+3.3V" H 10215 5773 50  0000 C CNN
F 2 "" H 10200 5600 50  0001 C CNN
F 3 "" H 10200 5600 50  0001 C CNN
	1    10200 5600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5F8F6380
P 6400 3550
F 0 "#PWR0103" H 6400 3400 50  0001 C CNN
F 1 "+3.3V" H 6415 3723 50  0000 C CNN
F 2 "" H 6400 3550 50  0001 C CNN
F 3 "" H 6400 3550 50  0001 C CNN
	1    6400 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5F8F72D1
P 2500 800
F 0 "#PWR0104" H 2500 650 50  0001 C CNN
F 1 "+3.3V" H 2515 973 50  0000 C CNN
F 2 "" H 2500 800 50  0001 C CNN
F 3 "" H 2500 800 50  0001 C CNN
	1    2500 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 5F8FDF1A
P 9900 4100
F 0 "#PWR0105" H 9900 3950 50  0001 C CNN
F 1 "+3.3V" H 9915 4273 50  0000 C CNN
F 2 "" H 9900 4100 50  0001 C CNN
F 3 "" H 9900 4100 50  0001 C CNN
	1    9900 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0106
U 1 1 5F8FE3B0
P 8900 2850
F 0 "#PWR0106" H 8900 2700 50  0001 C CNN
F 1 "+3.3V" H 8915 3023 50  0000 C CNN
F 2 "" H 8900 2850 50  0001 C CNN
F 3 "" H 8900 2850 50  0001 C CNN
	1    8900 2850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0107
U 1 1 5F900547
P 10850 2250
F 0 "#PWR0107" H 10850 2100 50  0001 C CNN
F 1 "+3.3V" H 10865 2423 50  0000 C CNN
F 2 "" H 10850 2250 50  0001 C CNN
F 3 "" H 10850 2250 50  0001 C CNN
	1    10850 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5F900A4C
P 9600 1000
F 0 "#PWR0108" H 9600 850 50  0001 C CNN
F 1 "+3.3V" H 9615 1173 50  0000 C CNN
F 2 "" H 9600 1000 50  0001 C CNN
F 3 "" H 9600 1000 50  0001 C CNN
	1    9600 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 5F906BFE
P 9450 4100
F 0 "#PWR0109" H 9450 3950 50  0001 C CNN
F 1 "+3.3V" H 9465 4273 50  0000 C CNN
F 2 "" H 9450 4100 50  0001 C CNN
F 3 "" H 9450 4100 50  0001 C CNN
	1    9450 4100
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint 3V3
U 1 1 5F90F1A7
P 10200 5600
F 0 "3V3" H 10142 5626 50  0000 R CNN
F 1 "TestPoint" H 10142 5717 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 10400 5600 50  0001 C CNN
F 3 "~" H 10400 5600 50  0001 C CNN
	1    10200 5600
	-1   0    0    1   
$EndComp
Connection ~ 10200 5600
$Comp
L Connector:TestPoint SCL1
U 1 1 5F914483
P 9100 1000
F 0 "SCL1" H 9158 1118 50  0000 L CNN
F 1 "TestPoint" H 9158 1027 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 9300 1000 50  0001 C CNN
F 3 "~" H 9300 1000 50  0001 C CNN
	1    9100 1000
	1    0    0    -1  
$EndComp
Connection ~ 9100 1000
Wire Wire Line
	9100 1000 9500 1000
$Comp
L Connector:TestPoint SDA1
U 1 1 5F916A4C
P 9050 1150
F 0 "SDA1" H 8992 1176 50  0000 R CNN
F 1 "TestPoint" H 8992 1267 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 9250 1150 50  0001 C CNN
F 3 "~" H 9250 1150 50  0001 C CNN
	1    9050 1150
	-1   0    0    1   
$EndComp
Connection ~ 9050 1150
Wire Wire Line
	9050 1150 9400 1150
$Comp
L Connector:TestPoint PB0
U 1 1 5F917152
P 3150 2000
F 0 "PB0" V 3104 2188 50  0000 L CNN
F 1 "TestPoint" V 3195 2188 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 3350 2000 50  0001 C CNN
F 3 "~" H 3350 2000 50  0001 C CNN
	1    3150 2000
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint PB1
U 1 1 5F919346
P 3150 2100
F 0 "PB1" V 3104 2288 50  0000 L CNN
F 1 "TestPoint" V 3195 2288 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 3350 2100 50  0001 C CNN
F 3 "~" H 3350 2100 50  0001 C CNN
	1    3150 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	3150 2000 3100 2000
Wire Wire Line
	3150 2100 3100 2100
$Comp
L Device:R_PHOTO R2
U 1 1 5F968D8E
P 4350 1150
F 0 "R2" H 4420 1196 50  0000 L CNN
F 1 "R_PHOTO" H 4420 1105 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P2.54mm_Vertical" V 4400 900 50  0001 L CNN
F 3 "~" H 4350 1100 50  0001 C CNN
	1    4350 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1800 3650 1800
Wire Wire Line
	4450 1550 4450 1650
Wire Wire Line
	4500 1550 4450 1550
Text GLabel 4500 1550 2    50   Input ~ 0
SCK_ISP
Wire Wire Line
	4500 1650 4450 1650
Text GLabel 4500 1650 2    50   Input ~ 0
SCL
Wire Wire Line
	3100 1500 4450 1500
Wire Wire Line
	4450 1500 4450 1550
Connection ~ 4450 1550
Wire Wire Line
	3100 1200 3800 1200
Wire Wire Line
	3800 1200 3800 1000
Wire Wire Line
	3800 1000 3950 1000
Connection ~ 3950 1000
$Comp
L Device:R R1
U 1 1 5F99FB00
P 4350 850
F 0 "R1" H 4420 896 50  0000 L CNN
F 1 "10k" H 4420 805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4280 850 50  0001 C CNN
F 3 "~" H 4350 850 50  0001 C CNN
	1    4350 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 700  3950 700 
Connection ~ 3950 700 
Wire Wire Line
	4350 1300 3950 1300
Wire Wire Line
	3950 1300 3100 1300
Connection ~ 3950 1300
Wire Wire Line
	3100 1400 4800 1400
Wire Wire Line
	4800 1400 4800 1000
Wire Wire Line
	4800 1000 4350 1000
Connection ~ 4350 1000
$EndSCHEMATC
