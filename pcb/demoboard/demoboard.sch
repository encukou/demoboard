EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x15 J5
U 1 1 5E2C34EC
P 2050 2750
F 0 "J5" H 2130 2792 50  0000 L CNN
F 1 "Conn_01x15" H 2130 2701 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 2050 2750 50  0001 C CNN
F 3 "~" H 2050 2750 50  0001 C CNN
	1    2050 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x15 J8
U 1 1 5E2C4E40
P 3200 2750
F 0 "J8" H 3280 2792 50  0000 L CNN
F 1 "Conn_01x15" H 3280 2701 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 3200 2750 50  0001 C CNN
F 3 "~" H 3200 2750 50  0001 C CNN
	1    3200 2750
	-1   0    0    -1  
$EndComp
$Comp
L Driver_Motor:L293D U2
U 1 1 5E2C5047
P 6100 2800
F 0 "U2" H 5700 3850 50  0000 C CNN
F 1 "L293D" H 6500 3800 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 6350 2050 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/l293.pdf" H 5800 3500 50  0001 C CNN
	1    6100 2800
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2803A U4
U 1 1 5E2C7F25
P 8550 3900
F 0 "U4" H 8550 4467 50  0000 C CNN
F 1 "ULN2803A" H 8550 4376 50  0000 C CNN
F 2 "Package_DIP:DIP-18_W7.62mm" H 8600 3250 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2803a.pdf" H 8650 3700 50  0001 C CNN
	1    8550 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 5E2C8F97
P 2150 5550
F 0 "J6" H 2230 5592 50  0000 L CNN
F 1 "WSLED" H 2230 5501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2150 5550 50  0001 C CNN
F 3 "~" H 2150 5550 50  0001 C CNN
	1    2150 5550
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J9
U 1 1 5E2C9C7D
P 3200 5550
F 0 "J9" H 3280 5592 50  0000 L CNN
F 1 "Servo1" H 3280 5501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3200 5550 50  0001 C CNN
F 3 "~" H 3200 5550 50  0001 C CNN
	1    3200 5550
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J11
U 1 1 5E2C9F69
P 4200 5500
F 0 "J11" H 4280 5542 50  0000 L CNN
F 1 "Servo2" H 4280 5451 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4200 5500 50  0001 C CNN
F 3 "~" H 4200 5500 50  0001 C CNN
	1    4200 5500
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J12
U 1 1 5E2CA539
P 6150 5500
F 0 "J12" H 6230 5542 50  0000 L CNN
F 1 "TEMP" H 6230 5451 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 6150 5500 50  0001 C CNN
F 3 "~" H 6150 5500 50  0001 C CNN
	1    6150 5500
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J19
U 1 1 5E2CC9BE
P 10000 3800
F 0 "J19" H 10080 3842 50  0000 L CNN
F 1 "STEPPER1" H 10080 3751 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 10000 3800 50  0001 C CNN
F 3 "~" H 10000 3800 50  0001 C CNN
	1    10000 3800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J20
U 1 1 5E2CD0D9
P 10000 4550
F 0 "J20" H 10080 4592 50  0000 L CNN
F 1 "STEPPER2" H 10080 4501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 10000 4550 50  0001 C CNN
F 3 "~" H 10000 4550 50  0001 C CNN
	1    10000 4550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5E2CE48D
P 1150 7000
F 0 "J2" H 1230 6992 50  0000 L CNN
F 1 "BAT" H 1230 6901 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 1150 7000 50  0001 C CNN
F 3 "~" H 1150 7000 50  0001 C CNN
	1    1150 7000
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J13
U 1 1 5E2D0346
P 7450 2300
F 0 "J13" H 7530 2292 50  0000 L CNN
F 1 "MOT_A" H 7530 2201 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 7450 2300 50  0001 C CNN
F 3 "~" H 7450 2300 50  0001 C CNN
	1    7450 2300
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J14
U 1 1 5E2D08D4
P 7450 2900
F 0 "J14" H 7530 2892 50  0000 L CNN
F 1 "MOT_B" H 7530 2801 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical" H 7450 2900 50  0001 C CNN
F 3 "~" H 7450 2900 50  0001 C CNN
	1    7450 2900
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J17
U 1 1 5E2D354A
P 9850 5500
F 0 "J17" H 9930 5542 50  0000 L CNN
F 1 "7SEG1" H 9930 5451 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 9850 5500 50  0001 C CNN
F 3 "~" H 9850 5500 50  0001 C CNN
	1    9850 5500
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J18
U 1 1 5E2D3B62
P 9850 6100
F 0 "J18" H 9930 6142 50  0000 L CNN
F 1 "7SEG2" H 9930 6051 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 9850 6100 50  0001 C CNN
F 3 "~" H 9850 6100 50  0001 C CNN
	1    9850 6100
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x15 J3
U 1 1 5E2D8C6B
P 1050 2750
F 0 "J3" H 1130 2792 50  0000 L CNN
F 1 "Conn_01x15" H 1130 2701 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 1050 2750 50  0001 C CNN
F 3 "~" H 1050 2750 50  0001 C CNN
	1    1050 2750
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x15 J10
U 1 1 5E2D9553
P 4300 2750
F 0 "J10" H 4380 2792 50  0000 L CNN
F 1 "Conn_01x15" H 4380 2701 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 4300 2750 50  0001 C CNN
F 3 "~" H 4300 2750 50  0001 C CNN
	1    4300 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5E2DB4BB
P 1450 4400
F 0 "#PWR01" H 1450 4150 50  0001 C CNN
F 1 "GND" H 1455 4227 50  0000 C CNN
F 2 "" H 1450 4400 50  0001 C CNN
F 3 "" H 1450 4400 50  0001 C CNN
	1    1450 4400
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR02
U 1 1 5E2DCFC4
P 2450 3950
F 0 "#PWR02" H 2450 3800 50  0001 C CNN
F 1 "VCC" H 2467 4123 50  0000 C CNN
F 2 "" H 2450 3950 50  0001 C CNN
F 3 "" H 2450 3950 50  0001 C CNN
	1    2450 3950
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 5E2DD5F2
P 1050 5500
F 0 "J1" H 1130 5542 50  0000 L CNN
F 1 "BTN" H 1130 5451 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Horizontal" H 1050 5500 50  0001 C CNN
F 3 "~" H 1050 5500 50  0001 C CNN
	1    1050 5500
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J22
U 1 1 5E38BBB8
P 4750 6600
F 0 "J22" H 4668 6917 50  0000 C CNN
F 1 "OLED" H 4668 6826 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 4750 6600 50  0001 C CNN
F 3 "~" H 4750 6600 50  0001 C CNN
	1    4750 6600
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J23
U 1 1 5E38C1B7
P 5150 5500
F 0 "J23" H 5230 5542 50  0000 L CNN
F 1 "Servo3" H 5230 5451 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5150 5500 50  0001 C CNN
F 3 "~" H 5150 5500 50  0001 C CNN
	1    5150 5500
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J24
U 1 1 5E38DCD1
P 5650 6650
F 0 "J24" H 5568 6867 50  0000 C CNN
F 1 "LIGHT" H 5568 6776 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 5650 6650 50  0001 C CNN
F 3 "~" H 5650 6650 50  0001 C CNN
	1    5650 6650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1250 2050 1450 2050
Wire Wire Line
	1250 2150 1850 2150
Wire Wire Line
	1250 2250 1850 2250
Wire Wire Line
	1250 2350 1850 2350
Wire Wire Line
	1250 2450 1850 2450
Wire Wire Line
	1250 2550 1850 2550
Wire Wire Line
	1250 2650 1850 2650
Wire Wire Line
	1250 2750 1850 2750
Wire Wire Line
	1250 2850 1850 2850
Wire Wire Line
	1250 2950 1850 2950
Wire Wire Line
	1250 3050 1850 3050
Wire Wire Line
	1250 3150 1850 3150
Wire Wire Line
	1250 3250 1850 3250
Wire Wire Line
	1250 3350 1850 3350
Wire Wire Line
	1250 3450 1850 3450
Wire Wire Line
	3400 2050 4100 2050
Wire Wire Line
	3400 2150 4100 2150
Wire Wire Line
	3400 2250 4100 2250
Wire Wire Line
	3400 2350 4100 2350
Wire Wire Line
	3400 2450 4100 2450
Wire Wire Line
	3400 2550 4100 2550
Wire Wire Line
	3400 2650 4100 2650
Wire Wire Line
	3400 2750 4100 2750
Wire Wire Line
	3400 2850 4100 2850
Wire Wire Line
	3400 2950 4100 2950
Wire Wire Line
	3400 3050 4100 3050
Wire Wire Line
	3400 3150 4100 3150
Wire Wire Line
	3400 3250 4100 3250
Wire Wire Line
	3400 3350 4100 3350
Wire Wire Line
	3400 3450 4100 3450
Wire Wire Line
	6600 2200 7250 2200
Wire Wire Line
	6600 2400 7250 2400
Wire Wire Line
	7250 2400 7250 2300
Wire Wire Line
	6600 2800 7250 2800
Wire Wire Line
	6600 3000 7250 3000
Wire Wire Line
	7250 3000 7250 2900
$Comp
L power:GND #PWR0101
U 1 1 5E397CEC
P 5900 3600
F 0 "#PWR0101" H 5900 3350 50  0001 C CNN
F 1 "GND" H 5905 3427 50  0000 C CNN
F 2 "" H 5900 3600 50  0001 C CNN
F 3 "" H 5900 3600 50  0001 C CNN
	1    5900 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5E39827D
P 6000 3600
F 0 "#PWR0102" H 6000 3350 50  0001 C CNN
F 1 "GND" H 6005 3427 50  0000 C CNN
F 2 "" H 6000 3600 50  0001 C CNN
F 3 "" H 6000 3600 50  0001 C CNN
	1    6000 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5E39881A
P 6200 3600
F 0 "#PWR0103" H 6200 3350 50  0001 C CNN
F 1 "GND" H 6205 3427 50  0000 C CNN
F 2 "" H 6200 3600 50  0001 C CNN
F 3 "" H 6200 3600 50  0001 C CNN
	1    6200 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5E398DC3
P 6300 3600
F 0 "#PWR0104" H 6300 3350 50  0001 C CNN
F 1 "GND" H 6305 3427 50  0000 C CNN
F 2 "" H 6300 3600 50  0001 C CNN
F 3 "" H 6300 3600 50  0001 C CNN
	1    6300 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 1750 10400 1750
Wire Wire Line
	9600 1850 10400 1850
Wire Wire Line
	9600 1950 10400 1950
Wire Wire Line
	9600 2050 10400 2050
Wire Wire Line
	9600 2150 10400 2150
Wire Wire Line
	9600 2250 10400 2250
Wire Wire Line
	9600 2350 10400 2350
Wire Wire Line
	9600 2450 10400 2450
Text Label 10400 1750 0    50   ~ 0
B0
Text Label 10400 1850 0    50   ~ 0
B1
Text Label 10400 1950 0    50   ~ 0
B2
Text Label 10400 2050 0    50   ~ 0
B3
Text Label 10400 2150 0    50   ~ 0
B4
Text Label 10400 2250 0    50   ~ 0
B5
Text Label 10400 2350 0    50   ~ 0
B6
Text Label 10400 2450 0    50   ~ 0
B7
Text Label 7600 3700 0    50   ~ 0
B0
Text Label 7600 3800 0    50   ~ 0
B1
Text Label 7600 3900 0    50   ~ 0
B2
Text Label 7600 4000 0    50   ~ 0
B3
Text Label 7600 4100 0    50   ~ 0
B4
Text Label 7600 4200 0    50   ~ 0
B5
Text Label 7600 4300 0    50   ~ 0
B6
Text Label 7600 4400 0    50   ~ 0
B7
Wire Wire Line
	7600 3700 8150 3700
Wire Wire Line
	7600 3800 8150 3800
Wire Wire Line
	7600 3900 8150 3900
Wire Wire Line
	7600 4000 8150 4000
Wire Wire Line
	7600 4100 8150 4100
Wire Wire Line
	7600 4200 8150 4200
Wire Wire Line
	7600 4300 8150 4300
Wire Wire Line
	7600 4400 8150 4400
Wire Wire Line
	8950 3700 9800 3700
Wire Wire Line
	8950 3800 9800 3800
Wire Wire Line
	8950 3900 9800 3900
Wire Wire Line
	8950 4000 9800 4000
Wire Wire Line
	8950 4100 9350 4100
Wire Wire Line
	8950 4200 9300 4200
Wire Wire Line
	8950 4300 9250 4300
Wire Wire Line
	8950 4400 9200 4400
Wire Wire Line
	9350 4450 9800 4450
Wire Wire Line
	9350 4100 9350 4450
Wire Wire Line
	9300 4550 9800 4550
Wire Wire Line
	9300 4200 9300 4550
Wire Wire Line
	9250 4650 9800 4650
Wire Wire Line
	9250 4300 9250 4650
Wire Wire Line
	9200 4750 9800 4750
Wire Wire Line
	9200 4400 9200 4750
$Comp
L power:GND #PWR0107
U 1 1 5E3D4A04
P 8550 4600
F 0 "#PWR0107" H 8550 4350 50  0001 C CNN
F 1 "GND" H 8555 4427 50  0000 C CNN
F 2 "" H 8550 4600 50  0001 C CNN
F 3 "" H 8550 4600 50  0001 C CNN
	1    8550 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5E3D7823
P 9200 2850
F 0 "#PWR0109" H 9200 2600 50  0001 C CNN
F 1 "GND" H 9205 2677 50  0000 C CNN
F 2 "" H 9200 2850 50  0001 C CNN
F 3 "" H 9200 2850 50  0001 C CNN
	1    9200 2850
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0110
U 1 1 5E3D7E31
P 9200 1550
F 0 "#PWR0110" H 9200 1400 50  0001 C CNN
F 1 "VCC" H 9217 1723 50  0000 C CNN
F 2 "" H 9200 1550 50  0001 C CNN
F 3 "" H 9200 1550 50  0001 C CNN
	1    9200 1550
	1    0    0    -1  
$EndComp
Text Label 3650 2050 0    50   ~ 0
VIN
Text Label 3650 2150 0    50   ~ 0
GND
Text Label 3650 2250 0    50   ~ 0
D13
Text Label 3650 2350 0    50   ~ 0
D12
Text Label 3650 2450 0    50   ~ 0
D14
Text Label 3650 2550 0    50   ~ 0
D27
Text Label 3650 2650 0    50   ~ 0
D26
Text Label 3650 2750 0    50   ~ 0
D25
Text Label 3650 2850 0    50   ~ 0
D33
Text Label 3650 2950 0    50   ~ 0
D32
Text Label 3650 3050 0    50   ~ 0
D35
Text Label 3650 3150 0    50   ~ 0
D34
Text Label 3650 3250 0    50   ~ 0
D39
Text Label 3650 3350 0    50   ~ 0
D36
Text Label 3650 3450 0    50   ~ 0
EN
$Comp
L power:VCC #PWR03
U 1 1 5E3DEF61
P 1450 2050
F 0 "#PWR03" H 1450 1900 50  0001 C CNN
F 1 "VCC" H 1467 2223 50  0000 C CNN
F 2 "" H 1450 2050 50  0001 C CNN
F 3 "" H 1450 2050 50  0001 C CNN
	1    1450 2050
	1    0    0    -1  
$EndComp
Connection ~ 1450 2050
Wire Wire Line
	1450 2050 1850 2050
Text Label 1450 2150 0    50   ~ 0
GND
Text Label 1450 2250 0    50   ~ 0
D15
Text Label 1450 2350 0    50   ~ 0
D2
Text Label 1450 2450 0    50   ~ 0
D4
Text Label 1450 2550 0    50   ~ 0
D16
Text Label 1450 2650 0    50   ~ 0
D17
Text Label 1450 2750 0    50   ~ 0
D5
Text Label 1450 2850 0    50   ~ 0
D18
Text Label 1450 2950 0    50   ~ 0
D19
Text Label 1450 3050 0    50   ~ 0
D21
Text Label 1450 3150 0    50   ~ 0
D3
Text Label 1450 3250 0    50   ~ 0
D1
Text Label 1450 3350 0    50   ~ 0
D22
Text Label 1450 3450 0    50   ~ 0
D23
Text Label 1250 5400 0    50   ~ 0
VCC
Text Label 1250 5500 0    50   ~ 0
GND
Text Label 1250 5600 0    50   ~ 0
BTN
Text Label 2350 5450 0    50   ~ 0
GND
Text Label 2350 5550 0    50   ~ 0
WS
Text Label 2350 5650 0    50   ~ 0
VIN
Wire Wire Line
	1450 4100 1450 4200
Connection ~ 1450 4200
Text Label 1450 4100 0    50   ~ 0
GND
Wire Wire Line
	2450 3950 2450 4100
Text Label 2750 4350 0    50   ~ 0
VCC
Text Label 3400 5450 0    50   ~ 0
GND
Text Label 3400 5550 0    50   ~ 0
VIN
Text Label 3400 5650 0    50   ~ 0
PWM
Text Label 4400 5400 0    50   ~ 0
GND
Text Label 4400 5500 0    50   ~ 0
VIN
Text Label 4400 5600 0    50   ~ 0
DIG1
Text Label 5350 5400 0    50   ~ 0
GND
Text Label 5350 5500 0    50   ~ 0
VIN
Text Label 5350 5600 0    50   ~ 0
DIG2
Text Label 6350 5400 0    50   ~ 0
GND
Text Label 6350 5500 0    50   ~ 0
TMP
Text Label 6350 5600 0    50   ~ 0
VCC
$Comp
L Device:R R1
U 1 1 5E3FF5D1
P 6750 5500
F 0 "R1" V 6543 5500 50  0000 C CNN
F 1 "4K7" V 6634 5500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6680 5500 50  0001 C CNN
F 3 "~" H 6750 5500 50  0001 C CNN
	1    6750 5500
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR06
U 1 1 5E3FFD3C
P 6900 5500
F 0 "#PWR06" H 6900 5350 50  0001 C CNN
F 1 "VCC" H 6917 5673 50  0000 C CNN
F 2 "" H 6900 5500 50  0001 C CNN
F 3 "" H 6900 5500 50  0001 C CNN
	1    6900 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5500 6350 5500
Text Label 8700 5550 0    50   ~ 0
LED
$Comp
L power:VCC #PWR07
U 1 1 5E40799C
P 8800 5250
F 0 "#PWR07" H 8800 5100 50  0001 C CNN
F 1 "VCC" H 8817 5423 50  0000 C CNN
F 2 "" H 8800 5250 50  0001 C CNN
F 3 "" H 8800 5250 50  0001 C CNN
	1    8800 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 5450 8700 5450
Text Label 10050 5300 0    50   ~ 0
B2
Text Label 10050 5400 0    50   ~ 0
B6
Text Label 10050 5500 0    50   ~ 0
B4
Text Label 10050 5600 0    50   ~ 0
B3
Text Label 10050 5700 0    50   ~ 0
B7
Text Label 10050 5900 0    50   ~ 0
B0
Text Label 10050 6000 0    50   ~ 0
B1
Text Label 10850 6100 0    50   ~ 0
DIG2
Text Label 10850 6200 0    50   ~ 0
DIG1
Text Label 10050 6300 0    50   ~ 0
B5
Text Label 4950 6500 0    50   ~ 0
GND
Text Label 4950 6600 0    50   ~ 0
VCC
Text Label 4950 6700 0    50   ~ 0
SCL
Text Label 4950 6800 0    50   ~ 0
SDA
Text Label 1700 3050 0    50   ~ 0
SDA
Text Label 1700 3350 0    50   ~ 0
SCL
Text Label 5850 6650 0    50   ~ 0
LGT
$Comp
L Device:R_POT RV1
U 1 1 5E4256FF
P 6250 6500
F 0 "RV1" V 6043 6500 50  0000 C CNN
F 1 "100K" V 6134 6500 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 6250 6500 50  0001 C CNN
F 3 "~" H 6250 6500 50  0001 C CNN
	1    6250 6500
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR05
U 1 1 5E42643D
P 6550 6500
F 0 "#PWR05" H 6550 6350 50  0001 C CNN
F 1 "VCC" H 6567 6673 50  0000 C CNN
F 2 "" H 6550 6500 50  0001 C CNN
F 3 "" H 6550 6500 50  0001 C CNN
	1    6550 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 6500 6400 6500
Wire Wire Line
	6250 6650 5850 6650
$Comp
L power:GND #PWR04
U 1 1 5E42B516
P 5850 6750
F 0 "#PWR04" H 5850 6500 50  0001 C CNN
F 1 "GND" H 5855 6577 50  0000 C CNN
F 2 "" H 5850 6750 50  0001 C CNN
F 3 "" H 5850 6750 50  0001 C CNN
	1    5850 6750
	1    0    0    -1  
$EndComp
Text Label 3750 6600 0    50   ~ 0
JX
Text Label 3750 6800 0    50   ~ 0
JY
$Comp
L Connector_Generic:Conn_01x04 J16
U 1 1 5E42D623
P 3550 6700
F 0 "J16" H 3468 7017 50  0000 C CNN
F 1 "ANALOG" H 3468 6926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3550 6700 50  0001 C CNN
F 3 "~" H 3550 6700 50  0001 C CNN
	1    3550 6700
	-1   0    0    -1  
$EndComp
Text Label 3750 6700 0    50   ~ 0
GND
Text Label 3750 6900 0    50   ~ 0
VCC
Text Label 1350 7100 0    50   ~ 0
VDC
Text Label 1350 7000 0    50   ~ 0
GND
$Comp
L power:VCC #PWR0111
U 1 1 5E431F08
P 6000 1800
F 0 "#PWR0111" H 6000 1650 50  0001 C CNN
F 1 "VCC" H 6017 1973 50  0000 C CNN
F 2 "" H 6000 1800 50  0001 C CNN
F 3 "" H 6000 1800 50  0001 C CNN
	1    6000 1800
	1    0    0    -1  
$EndComp
$Comp
L power:VDC #PWR0112
U 1 1 5E432823
P 6200 1800
F 0 "#PWR0112" H 6200 1700 50  0001 C CNN
F 1 "VDC" H 6200 2075 50  0000 C CNN
F 2 "" H 6200 1800 50  0001 C CNN
F 3 "" H 6200 1800 50  0001 C CNN
	1    6200 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 2200 5250 2200
Wire Wire Line
	5600 2400 5250 2400
Wire Wire Line
	5600 2600 5250 2600
Wire Wire Line
	5600 2800 5250 2800
Wire Wire Line
	5600 3000 5250 3000
Wire Wire Line
	5600 3200 5250 3200
Text Label 5250 2200 0    50   ~ 0
MA1
Text Label 5250 2400 0    50   ~ 0
MA2
Text Label 5250 2600 0    50   ~ 0
MAE
Text Label 5250 2800 0    50   ~ 0
MB1
Text Label 5250 3000 0    50   ~ 0
MB2
Text Label 5250 3200 0    50   ~ 0
MBE
Text Label 8400 1750 0    50   ~ 0
MOSI
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5E461E98
P 2750 4100
F 0 "#FLG0101" H 2750 4175 50  0001 C CNN
F 1 "PWR_FLAG" H 2750 4273 50  0000 C CNN
F 2 "" H 2750 4100 50  0001 C CNN
F 3 "~" H 2750 4100 50  0001 C CNN
	1    2750 4100
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5E4660E1
P 3300 4150
F 0 "#FLG0102" H 3300 4225 50  0001 C CNN
F 1 "PWR_FLAG" H 3300 4323 50  0000 C CNN
F 2 "" H 3300 4150 50  0001 C CNN
F 3 "~" H 3300 4150 50  0001 C CNN
	1    3300 4150
	1    0    0    -1  
$EndComp
$Comp
L power:VDC #PWR0113
U 1 1 5E46672A
P 3600 4150
F 0 "#PWR0113" H 3600 4050 50  0001 C CNN
F 1 "VDC" H 3600 4425 50  0000 C CNN
F 2 "" H 3600 4150 50  0001 C CNN
F 3 "" H 3600 4150 50  0001 C CNN
	1    3600 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4150 3300 4150
NoConn ~ 6100 6500
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5E47B61E
P 1850 4200
F 0 "#FLG0103" H 1850 4275 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 4373 50  0000 C CNN
F 2 "" H 1850 4200 50  0001 C CNN
F 3 "~" H 1850 4200 50  0001 C CNN
	1    1850 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4200 1450 4200
Wire Wire Line
	2450 4100 2750 4100
Wire Wire Line
	9800 4350 9600 4350
Wire Wire Line
	9800 3600 9600 3600
Text Label 1700 3250 0    50   ~ 0
TX
Text Label 1700 3150 0    50   ~ 0
RX
Text Label 1700 2450 0    50   ~ 0
TMP
Text Label 1700 2350 0    50   ~ 0
LED
Text Label 3900 2350 0    50   ~ 0
MA2
Text Label 3900 2250 0    50   ~ 0
PWM
Text Label 3900 2450 0    50   ~ 0
DIG1
Text Label 1700 2550 0    50   ~ 0
MBE
Text Label 1700 2650 0    50   ~ 0
MB1
Text Label 1700 2850 0    50   ~ 0
CLK
Text Label 1700 2950 0    50   ~ 0
MISO
Text Label 3900 2650 0    50   ~ 0
MAE
Text Label 3900 2750 0    50   ~ 0
DIG2
Text Label 1700 3450 0    50   ~ 0
MOSI
Text Label 3900 2550 0    50   ~ 0
MA1
Text Label 3900 2950 0    50   ~ 0
WS
Text Label 3900 2850 0    50   ~ 0
RCLK
Text Label 3900 3050 0    50   ~ 0
BTN
Text Label 3900 3150 0    50   ~ 0
TCH
Text Label 3900 3350 0    50   ~ 0
JX
Text Label 3900 3250 0    50   ~ 0
JY
Text Label 1700 2750 0    50   ~ 0
MB2
Text Label 1700 2250 0    50   ~ 0
LGT
Text Label 6800 2800 0    50   ~ 0
MB1Y
Text Label 6800 3000 0    50   ~ 0
MB2Y
Wire Wire Line
	1450 4200 1450 4400
Wire Wire Line
	2750 4150 2750 4350
$Comp
L Device:R R4
U 1 1 5E41F1B6
P 1950 6350
F 0 "R4" V 1743 6350 50  0000 C CNN
F 1 "100R" V 1834 6350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1880 6350 50  0001 C CNN
F 3 "~" H 1950 6350 50  0001 C CNN
	1    1950 6350
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5E41F835
P 2450 6350
F 0 "D1" H 2443 6566 50  0000 C CNN
F 1 "LED" H 2443 6475 50  0000 C CNN
F 2 "LED_THT:LED_D3.0mm_Horizontal_O1.27mm_Z6.0mm" H 2450 6350 50  0001 C CNN
F 3 "~" H 2450 6350 50  0001 C CNN
	1    2450 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 6350 1550 6350
Wire Wire Line
	2100 6350 2300 6350
Wire Wire Line
	2600 6350 2950 6350
$Comp
L power:GND #PWR0105
U 1 1 5E42C8D7
P 1550 6350
F 0 "#PWR0105" H 1550 6100 50  0001 C CNN
F 1 "GND" H 1555 6177 50  0000 C CNN
F 2 "" H 1550 6350 50  0001 C CNN
F 3 "" H 1550 6350 50  0001 C CNN
	1    1550 6350
	1    0    0    -1  
$EndComp
Text Label 2950 6350 0    50   ~ 0
D2
$Comp
L power:VDC #PWR0106
U 1 1 5E444DC5
P 9600 3600
F 0 "#PWR0106" H 9600 3500 50  0001 C CNN
F 1 "VDC" H 9600 3875 50  0000 C CNN
F 2 "" H 9600 3600 50  0001 C CNN
F 3 "" H 9600 3600 50  0001 C CNN
	1    9600 3600
	1    0    0    -1  
$EndComp
$Comp
L power:VDC #PWR0114
U 1 1 5E445402
P 9600 4350
F 0 "#PWR0114" H 9600 4250 50  0001 C CNN
F 1 "VDC" H 9600 4625 50  0000 C CNN
F 2 "" H 9600 4350 50  0001 C CNN
F 3 "" H 9600 4350 50  0001 C CNN
	1    9600 4350
	1    0    0    -1  
$EndComp
Connection ~ 9600 3600
Wire Wire Line
	8950 3600 9600 3600
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 5E44A6AB
P 8400 5450
F 0 "J4" H 8318 5025 50  0000 C CNN
F 1 "Conn_01x04" H 8318 5116 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Horizontal" H 8400 5450 50  0001 C CNN
F 3 "~" H 8400 5450 50  0001 C CNN
	1    8400 5450
	-1   0    0    1   
$EndComp
Wire Wire Line
	8800 5250 8600 5250
Wire Wire Line
	8600 5550 8700 5550
Wire Wire Line
	8600 5350 8700 5350
Text Label 8700 5350 0    50   ~ 0
BTN
$Comp
L 74xx:74HC595 U1
U 1 1 5E474ABC
P 9200 2150
F 0 "U1" H 8900 2750 50  0000 C CNN
F 1 "74HC595" H 9450 2750 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 9200 2150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 9200 2150 50  0001 C CNN
	1    9200 2150
	1    0    0    -1  
$EndComp
NoConn ~ 9600 2650
Wire Wire Line
	8400 1750 8800 1750
Wire Wire Line
	8400 1950 8800 1950
Text Label 8400 1950 0    50   ~ 0
CLK
$Comp
L power:GND #PWR0108
U 1 1 5E48BD34
P 8800 2350
F 0 "#PWR0108" H 8800 2100 50  0001 C CNN
F 1 "GND" H 8805 2177 50  0000 C CNN
F 2 "" H 8800 2350 50  0001 C CNN
F 3 "" H 8800 2350 50  0001 C CNN
	1    8800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 2250 8800 2250
Text Label 8400 2250 0    50   ~ 0
RCLK
Text Label 8400 2050 0    50   ~ 0
SRCLR
Text Label 8700 5450 0    50   ~ 0
TCH
Text Label 6800 2400 0    50   ~ 0
MA2Y
Text Label 6800 2200 0    50   ~ 0
MA1Y
$Comp
L Device:R R3
U 1 1 5E623362
P 10200 6100
F 0 "R3" V 10000 6200 50  0000 C CNN
F 1 "220R" V 10100 6200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 10130 6100 50  0001 C CNN
F 3 "~" H 10200 6100 50  0001 C CNN
	1    10200 6100
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E623A2A
P 10700 6200
F 0 "R2" V 10800 6200 50  0000 C CNN
F 1 "220R" V 10900 6200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 10630 6200 50  0001 C CNN
F 3 "~" H 10700 6200 50  0001 C CNN
	1    10700 6200
	0    1    1    0   
$EndComp
Wire Wire Line
	10050 6200 10550 6200
Wire Wire Line
	10350 6100 10850 6100
Wire Wire Line
	8250 2050 8800 2050
$Comp
L power:VCC #PWR0115
U 1 1 5E63193D
P 7800 2050
F 0 "#PWR0115" H 7800 1900 50  0001 C CNN
F 1 "VCC" H 7817 2223 50  0000 C CNN
F 2 "" H 7800 2050 50  0001 C CNN
F 3 "" H 7800 2050 50  0001 C CNN
	1    7800 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E686A6A
P 8100 2050
F 0 "R5" V 7893 2050 50  0000 C CNN
F 1 "4K7" V 7984 2050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8030 2050 50  0001 C CNN
F 3 "~" H 8100 2050 50  0001 C CNN
	1    8100 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	7950 2050 7800 2050
$EndSCHEMATC
