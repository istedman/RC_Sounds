EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
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
L RC_Sounds-rescue:DFR0299-DFR0299 U4
U 1 1 5F908738
P 8900 3100
F 0 "U4" H 8900 3967 50  0000 C CNN
F 1 "DFR0299" H 8900 3876 50  0000 C CNN
F 2 "DFR0299:MODULE_DFR0299" H 8900 3100 50  0001 L BNN
F 3 "" H 8900 3100 50  0001 L BNN
F 4 "DFRobot" H 8900 3100 50  0001 L BNN "Field4"
F 5 "None" H 8900 3100 50  0001 L BNN "Field5"
F 6 "DFR0299" H 8900 3100 50  0001 L BNN "Field6"
F 7 "Unavailable" H 8900 3100 50  0001 L BNN "Field7"
F 8 "Dfplayer - a Mini Mp3 Player" H 8900 3100 50  0001 L BNN "Field8"
	1    8900 3100
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LF50_TO252 U1
U 1 1 5F90E3A5
P 2600 1450
F 0 "U1" H 2600 1692 50  0000 C CNN
F 1 "LD29080DTR50" H 2600 1601 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:TO-252-2" H 2600 1675 50  0001 C CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/c4/0e/7e/2a/be/bc/4c/bd/CD00000546.pdf/files/CD00000546.pdf/jcr:content/translations/en.CD00000546.pdf" H 2600 1400 50  0001 C CNN
	1    2600 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:Polyfuse_Small F1
U 1 1 5F90F38B
P 1900 1450
F 0 "F1" H 1968 1496 50  0000 L CNN
F 1 "Polyfuse_Small" H 1968 1405 50  0000 L CNN
F 2 "Fuse_Holders_and_Fuses:Fuse_SMD1206_HandSoldering" H 1950 1250 50  0001 L CNN
F 3 "~" H 1900 1450 50  0001 C CNN
	1    1900 1450
	0    -1   -1   0   
$EndComp
$Comp
L Motor:Motor_Servo_JR M1
U 1 1 5F90FB10
P 2850 2650
F 0 "M1" H 2844 2994 50  0000 C CNN
F 1 "Motor_Servo_JR" H 2844 2903 50  0000 C CNN
F 2 "Connectors:1X03" H 2850 2460 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 2850 2460 50  0001 C CNN
	1    2850 2650
	-1   0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo_JR M2
U 1 1 5F912524
P 2950 4450
F 0 "M2" H 2944 4794 50  0000 C CNN
F 1 "Motor_Servo_JR" H 2944 4703 50  0000 C CNN
F 2 "Connectors:1X03" H 2950 4260 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 2950 4260 50  0001 C CNN
	1    2950 4450
	-1   0    0    -1  
$EndComp
$Comp
L RC_Sounds-rescue:PAM8403DR-PAM8403DR U3
U 1 1 5F918A9D
P 4750 8950
F 0 "U3" H 4750 9817 50  0000 C CNN
F 1 "PAM8403DR" H 4750 9726 50  0000 C CNN
F 2 "SMD_Packages:SO-16-N" H 4750 8950 50  0001 L BNN
F 3 "" H 4750 8950 50  0001 L BNN
	1    4750 8950
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C10
U 1 1 5F91B533
P 6400 8150
F 0 "C10" H 6518 8196 50  0000 L CNN
F 1 "470uF" H 6518 8105 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:CP_Tantalum_Case-E_EIA-7260-38_Hand" H 6438 8000 50  0001 C CNN
F 3 "~" H 6400 8150 50  0001 C CNN
	1    6400 8150
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C11
U 1 1 5F91CB6E
P 6900 8150
F 0 "C11" H 7018 8196 50  0000 L CNN
F 1 "1uF" H 7018 8105 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:CP_Tantalum_Case-A_EIA-3216-18_Hand" H 6938 8000 50  0001 C CNN
F 3 "~" H 6900 8150 50  0001 C CNN
	1    6900 8150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5F91DC34
P 2150 1700
F 0 "C1" H 2265 1746 50  0000 L CNN
F 1 "100nF" H 2265 1655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2188 1550 50  0001 C CNN
F 3 "~" H 2150 1700 50  0001 C CNN
	1    2150 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5F91E5B7
P 3900 9550
F 0 "C6" H 4015 9596 50  0000 L CNN
F 1 "1uF" H 4015 9505 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3938 9400 50  0001 C CNN
F 3 "~" H 3900 9550 50  0001 C CNN
	1    3900 9550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 5F91E909
P 10350 2100
F 0 "C17" H 10465 2146 50  0000 L CNN
F 1 "100nF" H 10465 2055 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10388 1950 50  0001 C CNN
F 3 "~" H 10350 2100 50  0001 C CNN
	1    10350 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5F91E959
P 2900 8650
F 0 "C2" V 3152 8650 50  0000 C CNN
F 1 "470nF" V 3061 8650 50  0000 C CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2938 8500 50  0001 C CNN
F 3 "~" H 2900 8650 50  0001 C CNN
	1    2900 8650
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C3
U 1 1 5F91ECDB
P 2900 9150
F 0 "C3" V 3152 9150 50  0000 C CNN
F 1 "470nF" V 3061 9150 50  0000 C CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2938 9000 50  0001 C CNN
F 3 "~" H 2900 9150 50  0001 C CNN
	1    2900 9150
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C5
U 1 1 5F91F011
P 3750 2800
F 0 "C5" H 3865 2846 50  0000 L CNN
F 1 "SPO" H 3865 2755 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3788 2650 50  0001 C CNN
F 3 "~" H 3750 2800 50  0001 C CNN
	1    3750 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5F91F428
P 5400 8050
F 0 "R7" H 5470 8096 50  0000 L CNN
F 1 "10K" H 5470 8005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5330 8050 50  0001 C CNN
F 3 "~" H 5400 8050 50  0001 C CNN
	1    5400 8050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5F9202D3
P 3300 9150
F 0 "R2" V 3507 9150 50  0000 C CNN
F 1 "10K" V 3416 9150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3230 9150 50  0001 C CNN
F 3 "~" H 3300 9150 50  0001 C CNN
	1    3300 9150
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5F9203DF
P 3300 8650
F 0 "R1" V 3507 8650 50  0000 C CNN
F 1 "10K" V 3416 8650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3230 8650 50  0001 C CNN
F 3 "~" H 3300 8650 50  0001 C CNN
	1    3300 8650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 5F920429
P 5400 2050
F 0 "R8" H 5470 2096 50  0000 L CNN
F 1 "10K" H 5470 2005 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5330 2050 50  0001 C CNN
F 3 "~" H 5400 2050 50  0001 C CNN
	1    5400 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 5F92088D
P 7850 3550
F 0 "R15" H 7920 3596 50  0000 L CNN
F 1 "1K" H 7920 3505 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7780 3550 50  0001 C CNN
F 3 "~" H 7850 3550 50  0001 C CNN
	1    7850 3550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F920AC0
P 3850 8350
F 0 "R5" H 3920 8396 50  0000 L CNN
F 1 "10K" H 3920 8305 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3780 8350 50  0001 C CNN
F 3 "~" H 3850 8350 50  0001 C CNN
	1    3850 8350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5F920C61
P 6500 5150
F 0 "R12" H 6570 5196 50  0000 L CNN
F 1 "10K" H 6570 5105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6430 5150 50  0001 C CNN
F 3 "~" H 6500 5150 50  0001 C CNN
	1    6500 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 5F920CDA
P 6150 5150
F 0 "R11" H 6220 5196 50  0000 L CNN
F 1 "22K" H 6220 5105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6080 5150 50  0001 C CNN
F 3 "~" H 6150 5150 50  0001 C CNN
	1    6150 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5F920EA0
P 5800 5150
F 0 "R10" H 5870 5196 50  0000 L CNN
F 1 "47K" H 5870 5105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5730 5150 50  0001 C CNN
F 3 "~" H 5800 5150 50  0001 C CNN
	1    5800 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5F920F72
P 3800 4350
F 0 "R4" H 3870 4396 50  0000 L CNN
F 1 "0" H 3870 4305 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3730 4350 50  0001 C CNN
F 3 "~" H 3800 4350 50  0001 C CNN
	1    3800 4350
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F9210C0
P 3500 2550
F 0 "R3" H 3570 2596 50  0000 L CNN
F 1 "0" H 3570 2505 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3430 2550 50  0001 C CNN
F 3 "~" H 3500 2550 50  0001 C CNN
	1    3500 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	2000 1450 2150 1450
$Comp
L power:GND #PWR01
U 1 1 5F925AE9
P 2600 2050
F 0 "#PWR01" H 2600 1800 50  0001 C CNN
F 1 "GND" H 2605 1877 50  0000 C CNN
F 2 "" H 2600 2050 50  0001 C CNN
F 3 "" H 2600 2050 50  0001 C CNN
	1    2600 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C4
U 1 1 5F92860C
P 3200 1750
F 0 "C4" H 3318 1796 50  0000 L CNN
F 1 "10uF" H 3318 1705 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:CP_Tantalum_Case-B_EIA-3528-21_Hand" H 3238 1600 50  0001 C CNN
F 3 "~" H 3200 1750 50  0001 C CNN
	1    3200 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1450 3200 1450
Wire Wire Line
	3200 1450 3200 1600
Wire Wire Line
	3200 1900 3200 2000
Wire Wire Line
	3200 2000 2600 2000
Wire Wire Line
	2600 1750 2600 2000
Connection ~ 2600 2000
Wire Wire Line
	2600 2000 2600 2050
Wire Wire Line
	2150 1850 2150 2000
Wire Wire Line
	2150 2000 2600 2000
Wire Wire Line
	2150 1550 2150 1450
Connection ~ 2150 1450
Wire Wire Line
	2150 1450 2300 1450
$Comp
L power:+5V #PWR02
U 1 1 5F92B3B4
P 3200 1450
F 0 "#PWR02" H 3200 1300 50  0001 C CNN
F 1 "+5V" H 3215 1623 50  0000 C CNN
F 2 "" H 3200 1450 50  0001 C CNN
F 3 "" H 3200 1450 50  0001 C CNN
	1    3200 1450
	1    0    0    -1  
$EndComp
Connection ~ 3200 1450
$Comp
L power:+5V #PWR016
U 1 1 5F93274A
P 7100 7850
F 0 "#PWR016" H 7100 7700 50  0001 C CNN
F 1 "+5V" V 7115 7978 50  0000 L CNN
F 2 "" H 7100 7850 50  0001 C CNN
F 3 "" H 7100 7850 50  0001 C CNN
	1    7100 7850
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 8000 6000 7850
Connection ~ 6000 7850
Wire Wire Line
	6000 7850 6400 7850
Wire Wire Line
	6400 8000 6400 7850
Connection ~ 6400 7850
Wire Wire Line
	6400 7850 6900 7850
Wire Wire Line
	6900 8000 6900 7850
Connection ~ 6900 7850
Wire Wire Line
	6900 7850 7100 7850
$Comp
L power:GND #PWR013
U 1 1 5F93C9FA
P 6050 9750
F 0 "#PWR013" H 6050 9500 50  0001 C CNN
F 1 "GND" H 6055 9577 50  0000 C CNN
F 2 "" H 6050 9750 50  0001 C CNN
F 3 "" H 6050 9750 50  0001 C CNN
	1    6050 9750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 8300 6400 8300
Connection ~ 6400 8300
Wire Wire Line
	6050 9750 6050 9650
Wire Wire Line
	5350 9450 6050 9450
Wire Wire Line
	5350 9550 6050 9550
Connection ~ 6050 9550
Wire Wire Line
	6050 9550 6050 9450
Wire Wire Line
	5350 9650 6050 9650
Connection ~ 6050 9650
Wire Wire Line
	6050 9650 6050 9550
$Comp
L power:GND #PWR06
U 1 1 5F9434D0
P 3900 9800
F 0 "#PWR06" H 3900 9550 50  0001 C CNN
F 1 "GND" H 3905 9627 50  0000 C CNN
F 2 "" H 3900 9800 50  0001 C CNN
F 3 "" H 3900 9800 50  0001 C CNN
	1    3900 9800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 9250 3900 9250
Wire Wire Line
	3900 9250 3900 9400
Wire Wire Line
	3900 9700 3900 9800
$Comp
L Device:Ferrite_Bead_Small FB3
U 1 1 5F954195
P 6750 8800
F 0 "FB3" V 6949 8800 33  0000 C CNN
F 1 "Ferrite_Bead_Small" V 6883 8800 33  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6680 8800 50  0001 C CNN
F 3 "~" H 6750 8800 50  0001 C CNN
	1    6750 8800
	0    -1   -1   0   
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB1
U 1 1 5F95470B
P 6350 9000
F 0 "FB1" V 6549 9000 33  0000 C CNN
F 1 "Ferrite_Bead_Small" V 6483 9000 33  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6280 9000 50  0001 C CNN
F 3 "~" H 6350 9000 50  0001 C CNN
	1    6350 9000
	0    -1   -1   0   
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB2
U 1 1 5F95497D
P 6650 9300
F 0 "FB2" V 6849 9300 33  0000 C CNN
F 1 "Ferrite_Bead_Small" V 6783 9300 33  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6580 9300 50  0001 C CNN
F 3 "~" H 6650 9300 50  0001 C CNN
	1    6650 9300
	0    -1   -1   0   
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB4
U 1 1 5F954CF3
P 7100 9150
F 0 "FB4" V 7299 9150 33  0000 C CNN
F 1 "Ferrite_Bead_Small" V 7233 9150 33  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7030 9150 50  0001 C CNN
F 3 "~" H 7100 9150 50  0001 C CNN
	1    7100 9150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 9050 6200 9050
Wire Wire Line
	6200 9050 6200 9150
Wire Wire Line
	6200 9150 7000 9150
Wire Wire Line
	5350 9150 6100 9150
Wire Wire Line
	6100 9150 6100 9300
Wire Wire Line
	6100 9300 6550 9300
Wire Wire Line
	5350 8850 6100 8850
Wire Wire Line
	6100 8850 6100 9000
Wire Wire Line
	6100 9000 6250 9000
Wire Wire Line
	5350 8750 6550 8750
Wire Wire Line
	6550 8750 6550 8800
Wire Wire Line
	6550 8800 6650 8800
$Comp
L Device:C C12
U 1 1 5F95E92D
P 7350 9700
F 0 "C12" H 7465 9746 50  0000 L CNN
F 1 "220pF" H 7465 9655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7388 9550 50  0001 C CNN
F 3 "~" H 7350 9700 50  0001 C CNN
	1    7350 9700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5F9602A5
P 7800 9700
F 0 "C13" H 7915 9746 50  0000 L CNN
F 1 "220pF" H 7915 9655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7838 9550 50  0001 C CNN
F 3 "~" H 7800 9700 50  0001 C CNN
	1    7800 9700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5F9604E3
P 8250 9700
F 0 "C14" H 8365 9746 50  0000 L CNN
F 1 "220pf" H 8365 9655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8288 9550 50  0001 C CNN
F 3 "~" H 8250 9700 50  0001 C CNN
	1    8250 9700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 5F960693
P 8700 9700
F 0 "C15" H 8815 9746 50  0000 L CNN
F 1 "220pF" H 8815 9655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8738 9550 50  0001 C CNN
F 3 "~" H 8700 9700 50  0001 C CNN
	1    8700 9700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5F961148
P 8000 10050
F 0 "#PWR017" H 8000 9800 50  0001 C CNN
F 1 "GND" H 8005 9877 50  0000 C CNN
F 2 "" H 8000 10050 50  0001 C CNN
F 3 "" H 8000 10050 50  0001 C CNN
	1    8000 10050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 9850 7350 9950
Wire Wire Line
	8700 9950 8700 9850
Wire Wire Line
	8250 9850 8250 9950
Wire Wire Line
	7800 9850 7800 9950
Wire Wire Line
	8000 10050 8000 9950
Connection ~ 8000 9950
Wire Wire Line
	6750 9300 7350 9300
Wire Wire Line
	7350 9300 7350 9550
Wire Wire Line
	7800 9150 7800 9550
Wire Wire Line
	8250 9000 8250 9550
Wire Wire Line
	8700 8800 8700 9550
Wire Wire Line
	6000 8300 6050 8300
Connection ~ 6050 8300
Wire Wire Line
	6050 8300 6400 8300
Wire Wire Line
	5400 7850 5400 7900
Wire Wire Line
	5400 7850 5850 7850
Wire Wire Line
	5400 8200 5400 9350
Wire Wire Line
	5400 9350 5350 9350
Wire Wire Line
	5350 8350 5850 8350
Wire Wire Line
	5850 8350 5850 7850
Connection ~ 5850 7850
Wire Wire Line
	5850 7850 6000 7850
Wire Wire Line
	5350 8450 5850 8450
Wire Wire Line
	5850 8450 5850 8350
Connection ~ 5850 8350
Wire Wire Line
	5350 8550 5850 8550
Wire Wire Line
	5850 8550 5850 8450
Connection ~ 5850 8450
$Comp
L RC_Sounds-rescue:CONN_02-Sparkfun_connectors J1
U 1 1 5F97EFE6
P 5250 10400
F 0 "J1" H 5208 10704 45  0000 C CNN
F 1 "CONN_02" H 5208 10620 45  0000 C CNN
F 2 "Connectors:1X02" H 5250 10650 20  0001 C CNN
F 3 "" H 5250 10400 50  0001 C CNN
F 4 "XXX-00000" H 5208 10631 60  0001 C CNN "Field4"
	1    5250 10400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5F980135
P 5450 10500
F 0 "#PWR011" H 5450 10250 50  0001 C CNN
F 1 "GND" H 5455 10327 50  0000 C CNN
F 2 "" H 5450 10500 50  0001 C CNN
F 3 "" H 5450 10500 50  0001 C CNN
	1    5450 10500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 10400 5450 10400
Wire Wire Line
	5450 10400 5450 10500
Wire Wire Line
	5350 10300 5400 10300
Wire Wire Line
	5400 10300 5400 9350
Connection ~ 5400 9350
Wire Wire Line
	5400 7850 3850 7850
Wire Wire Line
	3850 7850 3850 8200
Connection ~ 5400 7850
Wire Wire Line
	3850 8500 3850 8650
Wire Wire Line
	3850 8650 4150 8650
Text Notes 3700 10350 0    50   ~ 0
Fit to disable PAM8403 amplifier.
Wire Wire Line
	3800 9150 3800 8950
Wire Wire Line
	3800 8950 4150 8950
Wire Wire Line
	3800 8650 3800 8850
Wire Wire Line
	3800 8850 4150 8850
$Comp
L power:GND #PWR020
U 1 1 5F990D9E
P 10350 3950
F 0 "#PWR020" H 10350 3700 50  0001 C CNN
F 1 "GND" H 10355 3777 50  0000 C CNN
F 2 "" H 10350 3950 50  0001 C CNN
F 3 "" H 10350 3950 50  0001 C CNN
	1    10350 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 3150 10000 2700
Wire Wire Line
	10000 2700 9500 2700
$Comp
L power:GND #PWR019
U 1 1 5F9A1064
P 10150 2400
F 0 "#PWR019" H 10150 2150 50  0001 C CNN
F 1 "GND" H 10155 2227 50  0000 C CNN
F 2 "" H 10150 2400 50  0001 C CNN
F 3 "" H 10150 2400 50  0001 C CNN
	1    10150 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 2250 9900 2350
Wire Wire Line
	9650 1850 9900 1850
Wire Wire Line
	9900 1950 9900 1850
Connection ~ 9900 1850
Wire Wire Line
	9900 1850 10350 1850
Wire Wire Line
	9900 2350 10150 2350
Wire Wire Line
	10350 2350 10350 2250
Wire Wire Line
	10150 2400 10150 2350
Connection ~ 10150 2350
Wire Wire Line
	10150 2350 10350 2350
Wire Wire Line
	10350 1950 10350 1850
$Comp
L power:GND #PWR018
U 1 1 5F9BDD08
P 9700 4250
F 0 "#PWR018" H 9700 4000 50  0001 C CNN
F 1 "GND" H 9705 4077 50  0000 C CNN
F 2 "" H 9700 4250 50  0001 C CNN
F 3 "" H 9700 4250 50  0001 C CNN
	1    9700 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 3800 9700 3800
Wire Wire Line
	9700 3800 9700 4250
Connection ~ 8250 9950
Wire Wire Line
	8000 9950 8250 9950
Connection ~ 7800 9950
Wire Wire Line
	7800 9950 8000 9950
Wire Wire Line
	7350 9950 7800 9950
Wire Wire Line
	6850 8800 8700 8800
Wire Wire Line
	6450 9000 8250 9000
Wire Wire Line
	7200 9150 7800 9150
Wire Wire Line
	8250 9950 8700 9950
Wire Wire Line
	3150 9150 3050 9150
Wire Wire Line
	3150 8650 3050 8650
Wire Wire Line
	3450 8650 3800 8650
Wire Wire Line
	3450 9150 3800 9150
Text Notes 6650 10150 0    50   ~ 0
EMC filtering of class D output\n
$Comp
L Device:R R14
U 1 1 5F920701
P 7850 3300
F 0 "R14" H 7920 3346 50  0000 L CNN
F 1 "1K" H 7920 3255 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7780 3300 50  0001 C CNN
F 3 "~" H 7850 3300 50  0001 C CNN
	1    7850 3300
	0    1    -1   0   
$EndComp
Wire Wire Line
	8000 3550 8050 3550
Wire Wire Line
	9800 3000 9500 3000
Wire Wire Line
	7050 3350 7050 3300
Wire Wire Line
	8000 3300 8150 3300
Wire Wire Line
	9850 4100 9850 3100
Wire Wire Line
	9850 3100 9500 3100
Text Label 8700 4100 0    50   ~ 0
SW_RX
$Comp
L power:GND #PWR03
U 1 1 5FA4AF8E
P 3250 2900
F 0 "#PWR03" H 3250 2650 50  0001 C CNN
F 1 "GND" H 3255 2727 50  0000 C CNN
F 2 "" H 3250 2900 50  0001 C CNN
F 3 "" H 3250 2900 50  0001 C CNN
	1    3250 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2750 3250 2750
Wire Wire Line
	3250 2750 3250 2900
Wire Wire Line
	3650 2550 3750 2550
Wire Wire Line
	5250 2550 5250 3050
Wire Wire Line
	5250 3050 5100 3050
Wire Wire Line
	3200 1450 3650 1450
Wire Wire Line
	4500 1450 4500 2000
$Comp
L Device:C C8
U 1 1 5FA6A721
P 4700 2150
F 0 "C8" H 4815 2196 50  0000 L CNN
F 1 "100nF" H 4815 2105 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4738 2000 50  0001 C CNN
F 3 "~" H 4700 2150 50  0001 C CNN
	1    4700 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5FA6AE5F
P 4700 2400
F 0 "#PWR09" H 4700 2150 50  0001 C CNN
F 1 "GND" H 4705 2227 50  0000 C CNN
F 2 "" H 4700 2400 50  0001 C CNN
F 3 "" H 4700 2400 50  0001 C CNN
	1    4700 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2000 4500 2000
Connection ~ 4500 2000
Wire Wire Line
	4500 2000 4500 2750
Wire Wire Line
	4700 2400 4700 2300
Wire Wire Line
	3150 2550 3350 2550
$Comp
L power:GND #PWR04
U 1 1 5FA7987B
P 3350 4900
F 0 "#PWR04" H 3350 4650 50  0001 C CNN
F 1 "GND" H 3355 4727 50  0000 C CNN
F 2 "" H 3350 4900 50  0001 C CNN
F 3 "" H 3350 4900 50  0001 C CNN
	1    3350 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 4900 3350 4550
Wire Wire Line
	3350 4550 3250 4550
$Comp
L power:GND #PWR08
U 1 1 5FA80A68
P 4500 4050
F 0 "#PWR08" H 4500 3800 50  0001 C CNN
F 1 "GND" H 4505 3877 50  0000 C CNN
F 2 "" H 4500 4050 50  0001 C CNN
F 3 "" H 4500 4050 50  0001 C CNN
	1    4500 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4050 4500 3950
Wire Wire Line
	9500 2500 9650 2500
Wire Wire Line
	9650 1850 9650 2500
$Comp
L Device:C C7
U 1 1 5FAAD972
P 4150 4600
F 0 "C7" H 4265 4646 50  0000 L CNN
F 1 "SPO" H 4265 4555 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 4188 4450 50  0001 C CNN
F 3 "~" H 4150 4600 50  0001 C CNN
	1    4150 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5FAC9CDA
P 4150 4850
F 0 "#PWR07" H 4150 4600 50  0001 C CNN
F 1 "GND" H 4155 4677 50  0000 C CNN
F 2 "" H 4150 4850 50  0001 C CNN
F 3 "" H 4150 4850 50  0001 C CNN
	1    4150 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4350 3650 4350
Wire Wire Line
	3950 4350 4150 4350
Wire Wire Line
	4150 4450 4150 4350
Connection ~ 4150 4350
Wire Wire Line
	4150 4750 4150 4850
$Comp
L power:GND #PWR05
U 1 1 5FB09D3E
P 3750 3050
F 0 "#PWR05" H 3750 2800 50  0001 C CNN
F 1 "GND" H 3755 2877 50  0000 C CNN
F 2 "" H 3750 3050 50  0001 C CNN
F 3 "" H 3750 3050 50  0001 C CNN
	1    3750 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3450 7050 3550
Wire Wire Line
	9800 4000 9800 3000
Text Label 8550 4000 0    50   ~ 0
SW_TX
Connection ~ 8250 9000
Connection ~ 8700 8800
Wire Wire Line
	11150 2800 11150 7500
Wire Wire Line
	11300 3400 11300 3550
Wire Wire Line
	3750 2650 3750 2550
Connection ~ 3750 2550
Wire Wire Line
	3750 2550 5250 2550
Wire Wire Line
	3750 2950 3750 3050
Text Label 6500 3350 0    50   ~ 0
TX_MICRO
Text Label 6500 3450 0    50   ~ 0
RX_MICRO
$Comp
L power:GND #PWR021
U 1 1 5FB2F00F
P 12250 3450
F 0 "#PWR021" H 12250 3200 50  0001 C CNN
F 1 "GND" H 12255 3277 50  0000 C CNN
F 2 "" H 12250 3450 50  0001 C CNN
F 3 "" H 12250 3450 50  0001 C CNN
	1    12250 3450
	1    0    0    -1  
$EndComp
Text Label 11500 2800 0    50   ~ 0
L_AUDIO
Text Label 11500 3400 0    50   ~ 0
R_AUDIO
Text Notes 11550 2550 0    50   ~ 0
Non-amplified output for external Amp
Connection ~ 7800 9150
Connection ~ 7350 9300
$Comp
L RC_Sounds-rescue:AVR_SPI_PROG_3X2PTH-Sparkfun_connectors J2
U 1 1 5FBC0777
P 6300 2300
F 0 "J2" H 6300 2604 45  0000 C CNN
F 1 "AVR_SPI_PROG_3X2PTH" H 6300 2520 45  0000 C CNN
F 2 "Connectors:2X3" H 6300 2600 20  0001 C CNN
F 3 "" H 6300 2300 50  0001 C CNN
F 4 "XXX-00000" H 6300 2531 60  0001 C CNN "Field4"
	1    6300 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3150 5750 2200
Wire Wire Line
	5750 2200 6000 2200
Wire Wire Line
	5250 3050 6650 3050
Wire Wire Line
	6650 3050 6650 2300
Wire Wire Line
	6650 2300 6600 2300
Connection ~ 5250 3050
Wire Wire Line
	5850 3250 5850 2300
Wire Wire Line
	5850 2300 6000 2300
Wire Wire Line
	5950 3550 5950 2400
Wire Wire Line
	5950 2400 6000 2400
Wire Wire Line
	4500 1450 5400 1450
Wire Wire Line
	6700 1450 6700 2200
Wire Wire Line
	6700 2200 6600 2200
Connection ~ 4500 1450
Text Notes 4850 1150 0    50   ~ 0
PB5 Analogue in/switch\nPB2 LED\nPB1 (horn)\nPB0 (Aux in for horn)
$Comp
L RC_Sounds-rescue:ATtiny85-20SU-MCU_Microchip_ATtiny U2
U 1 1 5FA14CF5
P 4500 3350
F 0 "U2" H 3971 3396 50  0000 R CNN
F 1 "ATtiny85-20SU" H 3971 3305 50  0000 R CNN
F 2 "SMD_Packages:SOIC-8-N" H 4500 3350 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf" H 4500 3350 50  0001 C CNN
	1    4500 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3250 5850 3250
Wire Wire Line
	4150 4350 5150 4350
Wire Wire Line
	2350 7500 2350 8650
Wire Wire Line
	2350 8650 2750 8650
Wire Wire Line
	2150 7700 2150 9150
Wire Wire Line
	2150 9150 2750 9150
Wire Wire Line
	5400 1900 5400 1450
$Comp
L Device:R R9
U 1 1 5FD65B24
P 5400 5150
F 0 "R9" H 5470 5196 50  0000 L CNN
F 1 "100K" H 5470 5105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5330 5150 50  0001 C CNN
F 3 "~" H 5400 5150 50  0001 C CNN
	1    5400 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5FD707F9
P 6850 5150
F 0 "R13" H 6920 5196 50  0000 L CNN
F 1 "4.7K" H 6920 5105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6780 5150 50  0001 C CNN
F 3 "~" H 6850 5150 50  0001 C CNN
	1    6850 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5FD7CCD3
P 6100 6550
F 0 "#PWR014" H 6100 6300 50  0001 C CNN
F 1 "GND" H 6105 6377 50  0000 C CNN
F 2 "" H 6100 6550 50  0001 C CNN
F 3 "" H 6100 6550 50  0001 C CNN
	1    6100 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 6400 5950 6500
Wire Wire Line
	5950 6500 6050 6500
Wire Wire Line
	6250 6500 6250 6400
Wire Wire Line
	6150 6400 6150 6500
Connection ~ 6150 6500
Wire Wire Line
	6150 6500 6250 6500
Wire Wire Line
	6050 6400 6050 6500
Connection ~ 6050 6500
Wire Wire Line
	6050 6500 6100 6500
Wire Wire Line
	6100 6550 6100 6500
Connection ~ 6100 6500
Wire Wire Line
	6100 6500 6150 6500
$Comp
L RC_Sounds-rescue:MOMENTARY-SWITCH-SPST-SMD-5.2MM-SparkFun-Switches S1
U 1 1 5FE2E806
P 4950 5700
F 0 "S1" V 4992 5656 45  0000 R CNN
F 1 "MOMENTARY-SWITCH" V 4908 5656 45  0001 R CNN
F 2 "Switches:TACTILE_SWITCH_SMD_5.2MM" H 4950 5900 20  0001 C CNN
F 3 "" H 4950 5700 50  0001 C CNN
F 4 "SWCH-08247" V 4866 5656 60  0001 R CNN "Field4"
	1    4950 5700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5FE2FC63
P 4950 6100
F 0 "#PWR010" H 4950 5850 50  0001 C CNN
F 1 "GND" H 4955 5927 50  0000 C CNN
F 2 "" H 4950 6100 50  0001 C CNN
F 3 "" H 4950 6100 50  0001 C CNN
	1    4950 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5FE320AD
P 4950 5150
F 0 "R6" H 5020 5196 50  0000 L CNN
F 1 "2.2K" H 5020 5105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4880 5150 50  0001 C CNN
F 3 "~" H 4950 5150 50  0001 C CNN
	1    4950 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 5300 4950 5500
Wire Wire Line
	4950 5900 4950 6100
$Comp
L RC_Sounds-rescue:CONN_03-SparkFun-Connectors J3
U 1 1 5FE6A2C7
P 7150 4000
F 0 "J3" H 7108 4404 45  0000 C CNN
F 1 "CONN_03" H 7108 4320 45  0000 C CNN
F 2 "Connectors:1X03" H 7150 4400 20  0001 C CNN
F 3 "" H 7150 4000 50  0001 C CNN
F 4 "XXX-00000" H 7108 4331 60  0001 C CNN "Field4"
	1    7150 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 7500 11150 7500
Wire Wire Line
	2150 7700 11300 7700
Wire Wire Line
	8050 4000 9800 4000
Wire Wire Line
	8150 4100 9850 4100
Text Notes 4350 5700 0    50   ~ 0
Setup switch
Wire Wire Line
	6700 1450 7850 1450
Wire Wire Line
	9650 1450 9650 1850
Connection ~ 6700 1450
Connection ~ 9650 1850
$Comp
L Device:R R16
U 1 1 5FFBD55C
P 7850 1700
F 0 "R16" H 7920 1746 50  0000 L CNN
F 1 "470" H 7920 1655 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7780 1700 50  0001 C CNN
F 3 "~" H 7850 1700 50  0001 C CNN
	1    7850 1700
	1    0    0    1   
$EndComp
Wire Wire Line
	5850 3250 7050 3250
Wire Wire Line
	7050 3250 7050 2050
Connection ~ 5850 3250
Wire Wire Line
	7850 2050 7850 1850
Wire Wire Line
	7850 1550 7850 1450
Connection ~ 7850 1450
Wire Wire Line
	7850 1450 9650 1450
$Comp
L RC_Sounds-rescue:CONN_04-SparkFun-Connectors J6
U 1 1 6001321B
P 12500 2900
F 0 "J6" H 12272 2955 45  0000 R CNN
F 1 "CONN_04" H 12272 3039 45  0000 R CNN
F 2 "Connectors:1X04" H 12500 3400 20  0001 C CNN
F 3 "" H 12500 2900 50  0001 C CNN
F 4 "CONN-09696" H 12272 3134 60  0000 R CNN "Field4"
	1    12500 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	12200 2800 12200 2900
Wire Wire Line
	12200 2900 12400 2900
Wire Wire Line
	11150 2800 12200 2800
Wire Wire Line
	12200 3400 12200 3200
Wire Wire Line
	12200 3200 12400 3200
Wire Wire Line
	11300 3400 12200 3400
Wire Wire Line
	12400 3100 12250 3100
Wire Wire Line
	12250 3100 12250 3450
Wire Wire Line
	12400 3000 12250 3000
Wire Wire Line
	12250 3000 12250 3100
Connection ~ 12250 3100
$Comp
L RC_Sounds-rescue:CONN_04-SparkFun-Connectors J5
U 1 1 60056A94
P 10550 8900
F 0 "J5" H 10322 9008 45  0000 R CNN
F 1 "CONN_04" H 10322 9092 45  0000 R CNN
F 2 "Connectors:SCREWTERMINAL-3.5MM-4" H 10550 9400 20  0001 C CNN
F 3 "" H 10550 8900 50  0001 C CNN
F 4 "CONN-09696" H 10322 9134 60  0001 R CNN "Field4"
	1    10550 8900
	-1   0    0    1   
$EndComp
Wire Wire Line
	10350 8800 10350 8900
Wire Wire Line
	10350 8900 10450 8900
Wire Wire Line
	8700 8800 10350 8800
Wire Wire Line
	8250 9000 10450 9000
Wire Wire Line
	10350 9150 10350 9100
Wire Wire Line
	10350 9100 10450 9100
Wire Wire Line
	7800 9150 10350 9150
Wire Wire Line
	10350 9300 10350 9200
Wire Wire Line
	10350 9200 10450 9200
Wire Wire Line
	7350 9300 10350 9300
$Comp
L RC_Sounds-rescue:CONN_02-SparkFun-Connectors J4
U 1 1 600ACA5E
P 1400 1550
F 0 "J4" H 1358 1854 45  0000 C CNN
F 1 "CONN_02" H 1358 1770 45  0000 C CNN
F 2 "Connectors:SCREWTERMINAL-3.5MM-2" H 1400 1800 20  0001 C CNN
F 3 "" H 1400 1550 50  0001 C CNN
F 4 "XXX-00000" H 1358 1781 60  0001 C CNN "Field4"
	1    1400 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1450 1800 1450
Wire Wire Line
	1500 1550 1650 1550
Wire Wire Line
	1650 1550 1650 2000
Wire Wire Line
	1650 2000 2150 2000
Connection ~ 2150 2000
Wire Wire Line
	6050 8300 6050 9450
Connection ~ 6050 9450
Text Notes 1800 2650 0    50   ~ 0
Horn/hooter input
Text Notes 1650 4450 0    50   ~ 0
Optional speed/aux input
$Comp
L RC_Sounds-rescue:LED-GREEN0603-SparkFun-LED D1
U 1 1 60137D2D
P 7550 2050
F 0 "D1" V 7261 2000 45  0000 C CNN
F 1 "LED-GREEN0603" V 7345 2000 45  0000 C CNN
F 2 "LED:LED-0603" V 7350 2050 20  0001 C CNN
F 3 "" H 7550 2050 50  0001 C CNN
F 4 "DIO-00821" V 7334 2000 60  0001 C CNN "Field4"
	1    7550 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	7650 2050 7850 2050
Wire Wire Line
	7050 2050 7350 2050
$Comp
L RC_Sounds-rescue:PTV112-PTV112 R17
U 1 1 60199D6D
P 10350 3550
F 0 "R17" H 10280 3596 50  0000 R CNN
F 1 "PTV112" H 10280 3505 50  0000 R CNN
F 2 "DFR0299:PTV112" H 10350 3550 50  0001 L BNN
F 3 "" H 10350 3550 50  0001 L BNN
	1    10350 3550
	1    0    0    -1  
$EndComp
$Comp
L RC_Sounds-rescue:PTV112-PTV112 R17
U 2 1 6019E190
P 10650 2800
F 0 "R17" H 10580 2846 50  0000 R CNN
F 1 "PTV112" H 10580 2755 50  0000 R CNN
F 2 "DFR0299:PTV112" H 10650 2800 50  0001 L BNN
F 3 "" H 10650 2800 50  0001 L BNN
	2    10650 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 3850 10350 3950
Connection ~ 10350 3950
Wire Wire Line
	10650 3950 10650 3100
Wire Wire Line
	10350 3950 10650 3950
Wire Wire Line
	10550 3550 11300 3550
Connection ~ 11300 3550
Wire Wire Line
	11300 3550 11300 7700
Wire Wire Line
	10850 2800 11150 2800
Connection ~ 11150 2800
Wire Wire Line
	10000 3150 10350 3150
Wire Wire Line
	10350 3150 10350 3250
Wire Wire Line
	9500 2800 10350 2800
Wire Wire Line
	10350 2800 10350 2450
Wire Wire Line
	10350 2450 10650 2450
Wire Wire Line
	10650 2450 10650 2500
$Comp
L RC_Sounds-rescue:STAND-OFFTIGHT-SparkFun-Hardware H1
U 1 1 602F0EFC
P 15000 9000
F 0 "H1" H 15000 9100 45  0001 C CNN
F 1 "STAND-OFFTIGHT" H 15000 8900 45  0001 C CNN
F 2 "Hardware:STAND-OFF-TIGHT" H 15000 9150 20  0001 C CNN
F 3 "" H 15000 9000 50  0001 C CNN
F 4 "XXX-00000" H 15078 9000 60  0000 L CNN "Field4"
	1    15000 9000
	1    0    0    -1  
$EndComp
$Comp
L RC_Sounds-rescue:STAND-OFFTIGHT-SparkFun-Hardware H2
U 1 1 602F1C12
P 15000 9150
F 0 "H2" H 15000 9250 45  0001 C CNN
F 1 "STAND-OFFTIGHT" H 15000 9050 45  0001 C CNN
F 2 "Hardware:STAND-OFF-TIGHT" H 15000 9300 20  0001 C CNN
F 3 "" H 15000 9150 50  0001 C CNN
F 4 "XXX-00000" H 15078 9150 60  0000 L CNN "Field4"
	1    15000 9150
	1    0    0    -1  
$EndComp
$Comp
L RC_Sounds-rescue:STAND-OFFTIGHT-SparkFun-Hardware H3
U 1 1 602F1E75
P 15000 9300
F 0 "H3" H 15000 9400 45  0001 C CNN
F 1 "STAND-OFFTIGHT" H 15000 9200 45  0001 C CNN
F 2 "Hardware:STAND-OFF-TIGHT" H 15000 9450 20  0001 C CNN
F 3 "" H 15000 9300 50  0001 C CNN
F 4 "XXX-00000" H 15078 9300 60  0000 L CNN "Field4"
	1    15000 9300
	1    0    0    -1  
$EndComp
$Comp
L RC_Sounds-rescue:STAND-OFFTIGHT-SparkFun-Hardware H4
U 1 1 602F1FF8
P 15000 9450
F 0 "H4" H 15000 9550 45  0001 C CNN
F 1 "STAND-OFFTIGHT" H 15000 9350 45  0001 C CNN
F 2 "Hardware:STAND-OFF-TIGHT" H 15000 9600 20  0001 C CNN
F 3 "" H 15000 9450 50  0001 C CNN
F 4 "XXX-00000" H 15078 9450 60  0000 L CNN "Field4"
	1    15000 9450
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C16
U 1 1 602F58BB
P 9900 2100
F 0 "C16" H 10018 2146 50  0000 L CNN
F 1 "10uF" H 10018 2055 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:CP_Tantalum_Case-B_EIA-3528-21_Hand" H 9938 1950 50  0001 C CNN
F 3 "~" H 9900 2100 50  0001 C CNN
	1    9900 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 602F7734
P 6000 8150
F 0 "C9" H 6115 8196 50  0000 L CNN
F 1 "1uF" H 6115 8105 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 6038 8000 50  0001 C CNN
F 3 "~" H 6000 8150 50  0001 C CNN
	1    6000 8150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 60305474
P 6850 2550
F 0 "#PWR0101" H 6850 2300 50  0001 C CNN
F 1 "GND" H 6855 2377 50  0000 C CNN
F 2 "" H 6850 2550 50  0001 C CNN
F 3 "" H 6850 2550 50  0001 C CNN
	1    6850 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 2550 6850 2400
Wire Wire Line
	6850 2400 6600 2400
$Comp
L Graphic:Logo_Open_Hardware_Large #LOGO1
U 1 1 60319B7F
P 15250 8350
F 0 "#LOGO1" H 15250 8850 50  0001 C CNN
F 1 "Logo_Open_Hardware_Large" H 15250 7950 50  0001 C CNN
F 2 "" H 15250 8350 50  0001 C CNN
F 3 "~" H 15250 8350 50  0001 C CNN
	1    15250 8350
	1    0    0    -1  
$EndComp
$Comp
L RC_Sounds-rescue:CONN_02-Sparkfun_connectors J7
U 1 1 5FC3DC7E
P 3900 1900
F 0 "J7" H 3858 2204 45  0000 C CNN
F 1 "CONN_02" H 3858 2120 45  0000 C CNN
F 2 "Connectors:1X02" H 3900 2150 20  0001 C CNN
F 3 "" H 3900 1900 50  0001 C CNN
F 4 "XXX-00000" H 3858 2131 60  0001 C CNN "Field4"
	1    3900 1900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3150 2650 3300 2650
Wire Wire Line
	3300 2650 3300 2000
Wire Wire Line
	3300 2000 3800 2000
Wire Wire Line
	3800 1900 3650 1900
Wire Wire Line
	3650 1900 3650 1450
Connection ~ 3650 1450
Wire Wire Line
	3650 1450 4500 1450
Text Notes 3650 1150 0    50   ~ 0
J7 allows receiver BEC\noutput to power the circuit
Text Notes 6400 3700 0    50   ~ 0
Serial debug
Wire Wire Line
	5100 3450 7050 3450
Wire Wire Line
	5100 3350 7050 3350
Wire Wire Line
	7250 3900 7500 3900
Wire Wire Line
	7250 3800 7600 3800
Text Label 9250 8800 0    50   ~ 0
LEFT+
Text Label 9250 9000 0    50   ~ 0
LEFT-
Text Label 9250 9150 0    50   ~ 0
RIGHT-
Text Label 9250 9300 0    50   ~ 0
RIGHT+
Wire Wire Line
	8150 3300 8150 4100
Wire Wire Line
	8050 3550 8050 4000
Wire Wire Line
	7600 3800 7600 3300
Wire Wire Line
	7050 3300 7600 3300
Connection ~ 7600 3300
Wire Wire Line
	7600 3300 7700 3300
Wire Wire Line
	7050 3550 7500 3550
Wire Wire Line
	5100 3550 5400 3550
Wire Wire Line
	5100 3150 5150 3150
Wire Wire Line
	5150 4350 5150 3150
Connection ~ 5150 3150
Wire Wire Line
	5150 3150 5750 3150
$Comp
L Switch:SW_DIP_x04 SW1
U 1 1 5FB9525F
P 6150 6100
F 0 "SW1" V 6196 5870 50  0000 R CNN
F 1 "SW_DIP_x04" V 6105 5870 50  0000 R CNN
F 2 "Switches:DIPSWITCH-04-SMD" H 6150 6100 50  0001 C CNN
F 3 "~" H 6150 6100 50  0001 C CNN
	1    6150 6100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 5300 5800 5700
Wire Wire Line
	5800 5700 5950 5700
Wire Wire Line
	5950 5700 5950 5800
Wire Wire Line
	6150 5300 6150 5450
Wire Wire Line
	6150 5450 6050 5450
Wire Wire Line
	6050 5450 6050 5800
Wire Wire Line
	6150 5800 6150 5550
Wire Wire Line
	6150 5550 6500 5550
Wire Wire Line
	6500 5550 6500 5300
Wire Wire Line
	6250 5800 6250 5700
Wire Wire Line
	6250 5700 6850 5700
Wire Wire Line
	6850 5700 6850 5300
Wire Wire Line
	5400 5300 5400 6500
Wire Wire Line
	5400 6500 5950 6500
Connection ~ 5950 6500
Connection ~ 5400 1450
Wire Wire Line
	5400 1450 6700 1450
Wire Wire Line
	5400 5000 5400 4800
Wire Wire Line
	5800 5000 5800 4800
Wire Wire Line
	5800 4800 5400 4800
Connection ~ 5400 4800
Wire Wire Line
	5800 4800 6150 4800
Wire Wire Line
	6850 4800 6850 5000
Connection ~ 5800 4800
Wire Wire Line
	6500 5000 6500 4800
Connection ~ 6500 4800
Wire Wire Line
	6500 4800 6850 4800
Wire Wire Line
	6150 5000 6150 4800
Connection ~ 6150 4800
Wire Wire Line
	6150 4800 6500 4800
Wire Wire Line
	4950 4500 5400 4500
Wire Wire Line
	4950 4500 4950 5000
Wire Wire Line
	5400 2200 5400 3550
Connection ~ 5400 4500
Wire Wire Line
	5400 4500 5400 4800
$Comp
L power:GND #PWR0102
U 1 1 60A2B37A
P 7600 4100
F 0 "#PWR0102" H 7600 3850 50  0001 C CNN
F 1 "GND" H 7605 3927 50  0000 C CNN
F 2 "" H 7600 4100 50  0001 C CNN
F 3 "" H 7600 4100 50  0001 C CNN
	1    7600 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 4000 7600 4000
Wire Wire Line
	7600 4000 7600 4100
Wire Wire Line
	7500 3900 7500 3550
Connection ~ 7500 3550
Wire Wire Line
	7500 3550 7700 3550
Connection ~ 5400 3550
Wire Wire Line
	5400 3550 5950 3550
Wire Wire Line
	5400 3550 5400 4500
Text Notes 5400 3650 0    50   ~ 0
Missing on V1.1 PCB\n
$EndSCHEMATC
