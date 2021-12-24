""" ############################################################################################ """
""" ############################################################################################ """
""" CNC mill with Teensy4.0 """
""" V1_00, 2021-12-24, are """
""" ############################################################################################ """
""" ############################################################################################ """

"""
##### I2C Master --> Slave, Struct I2C recive, max. 32 Byte possible
Raspberry → Teensy, Byte 0..27, 28 Byte
Byte 0
  Bit 0 = Operate (0 - Stepper disabled 1 - Stepper enabled)
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 1
  Bit 0 = ActivateCommand
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 2
  Bit 0 = Manual command Spindle on
  Bit 1 = Manual command Coolant on
  Bit 2 = Manual command Clamp on
  Bit 3 = Manual command Light on
  Bit 4 = Manual command X+
  Bit 5 = Manual command X-
  Bit 6 = Manual command Y+
  Bit 7 = Manual command Y-
Byte 3
  Bit 0 = Manual command Z+
  Bit 1 = Manual command Z-
  Bit 2 = ClearDiagnosticCounterI2C
  Bit 3 = Measure tool length
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 4, 5 Spindle speed in manual mode
Byte 6  G Code / M Code
Byte 7, 8, 9, 10 Wert 0 (mit vier Nachkommastellen)
Byte 11, 12, 13, 14 Wert 1 (mit vier Nachkommastellen)
Byte 15, 16, 17, 18 Wert 2 (mit vier Nachkommastellen)
Byte 19, 20, 21, 22 Wert 3 (mit vier Nachkommastellen)
Byte 23, 24, 25, 26 Wert 4 (mit vier Nachkommastellen)
Byte 27 Checksum (XOR Byte 0..26)

##### Slave --> I2C Master, Struct I2C send, max. 32 Byte possible
Teensy → Raspberry, Byte 0..22, 23 Byte
Byte 0
  Bit 0 = OperateOn
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 1
  Bit 0 = CommandDone
  Bit 1 = ReadyForCommand
  Bit 2 = ProgramDone
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 2
  Bit 0 = Reference sensor X
  Bit 1 = Reference sensor Y
  Bit 2 = Reference sensor Z
  Bit 3 = Emergency stop button
  Bit 4 = Spindle on
  Bit 5 = Coolant on
  Bit 6 = Clamp on
  Bit 7 = Light on
Byte 3
  Bit 0 = Tool length sensor
  Bit 1 = Measure tool length done
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 4, 5, 6, 7   Actual position motor 0 (mit vier Nachkommastellen)
Byte 8, 9, 10, 11 Actual position motor 1 (mit vier Nachkommastellen)
Byte 12, 13, 14, 15 Actual position motor 2 (mit vier Nachkommastellen)
Byte 16, 17, 18, 19 Actual position motor 3 (mit vier Nachkommastellen)
Byte 20, 21 ActualStepProgram
Byte 22 Checksum (XOR Byte 0..21)
"""

""" ############################################################################################ """
""" Import python modules """
import smbus # I2C bus

""" ############################################################################################ """
""" Define I2C bus """
I2C_BUS_3 = 3

"""
Edit config.txt
sudo nano /boot/config.txt

Available I2C buses: 1, 3, 4, 5, 6, 7
Don't use bus 0 and 2

Bus 1 = Pin 3 (GPIO2) SDA1
        Pin 5 (GPIO3) SCL1
Bus 1 with internal pull up resistors, 3.3V

Bus 3 = Pin 11 (GPIO17) SDA
        Pin 13 (GPIO27) SCL 
Bus 3 without internal pull up resistors, 3.3V
dtoverlay=i2c-gpio,bus=3,i2c_gpio_delay_us=2,i2c_gpio_sda=17,i2c_gpio_scl=27

Bus 4 = Pin 16 (GPIO23) SDA
        Pin 18 (GPIO24) SCL 
Bus 4 without internal pull up resistors, 3.3V
dtoverlay=i2c-gpio,bus=4,i2c_gpio_delay_us=2,i2c_gpio_sda=23,i2c_gpio_scl=24

config.txt
Uncomment some or all of these to enable the optional hardware interfaces
dtparam=i2c_arm=on
dtparam=i2c1=on
#dtparam=i2s=on
#dtparam=spi=on

dtoverlay=i2c-gpio,bus=3,i2c_gpio_delay_us=2,i2c_gpio_sda=17,i2c_gpio_scl=27
dtoverlay=i2c-gpio,bus=4,i2c_gpio_delay_us=2,i2c_gpio_sda=23,i2c_gpio_scl=24
"""


""" ############################################################################################ """
""" Define I2C addresses """
I2C_ADDR_Teensy = 0x08

""" Initialize I2C """
bus_3 = smbus.SMBus(I2C_BUS_3) # Get I2C bus 3


""" ############################################################################################ """
""" Global variables """
# Read data I2C, Teensy → Raspberry
OperateOn = False # Bit data Byte 0
Reserve0_1 = False
Reserve0_2 = False
Reserve0_3 = False
Reserve0_4 = False
Reserve0_5 = False
Reserve0_6 = False
Reserve0_7 = False
CommandDone = False # Bit data Byte 1
ReadyForCommand = False
ProgramDone = False
Reserve1_3 = False
Reserve1_4 = False
Reserve1_5 = False
Reserve1_6 = False
Reserve1_7 = False
ReferenceSensorX = False # Bit data Byte 2
ReferenceSensorY = False
ReferenceSensorZ = False
EmergencyStopButton = False
SpindleOn = False
CoolantOn = False
ClampOn = False
LightOn = False
ToolLengthSensor = False # Bit data Byte 3
MeasureToolLengthDone = False
Reserve3_2 = False
Reserve3_3 = False
Reserve3_4 = False
Reserve3_5 = False
Reserve3_6 = False
Reserve3_7 = False
ReadDataMotor = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, # 23 Byte read data
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0]
CounterReadErrorI2C = 0
ReadDataAreValid = False
ChecksumRead = 0

# Write data I2C, Raspberry → Teensy
WriteByte0BitData = 0b00000000
WriteByte1BitData = 0b00000000
WriteByte2BitData = 0b00000000
WriteByte3BitData = 0b00000000
WriteByte4 = 0 # Spindle speed manual mode
WriteByte5 = 0 # Spindle speed manual mode
WriteByte6GCodeMCode = 0
WriteByte7 = 0
WriteByte8 = 0
WriteByte9 = 0
WriteByte10 = 0
WriteByte11 = 0
WriteByte12 = 0
WriteByte13 = 0
WriteByte14 = 0
WriteByte15 = 0
WriteByte16 = 0
WriteByte17 = 0
WriteByte18 = 0
WriteByte19 = 0
WriteByte20 = 0
WriteByte21 = 0
WriteByte22 = 0
WriteByte23 = 0
WriteByte24 = 0
WriteByte25 = 0
WriteByte26 = 0
WriteByte27Checksum = 0
WriteDataMotor = [0b00000000, 0b00000000, 0b00000000, 0b00000000, # 26 byte write data
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0]
CounterWriteErrorI2C = 0
WriteDataAreValid = False

# I2C statistic data
CounterI2CReciveGood = 0
CounterI2CReciveChecksumError = 0
CounterI2CReciveHardwareError = 0
CounterI2CSendGood = 0
CounterI2CSendHardwareError = 0

# Common Variables
ActualStepProgram = 0
DecValueByte = [0, 0, 0, 0]
InchingActive = False
ManualMode = False
MeasureToolLength = False

# Stepvariables
CaseIdle = 10
CaseTransferFilename = 20
CaseReadFile = 30
CaseTransferFile = 40
CaseMotHandshakeCommandDone = 50
CaseMotOperateOn = 60
CaseMotOperateOff = 70
CaseMotInching = 80
CaseMeasureToolLength = 90
StepNoMotor = CaseIdle

# Print actual step
CompareMemoryActualStep = 9999

# Class move parameter
class MoveParameter:
    GCode = 0
    Value0 = 0
    Value1 = 0
    Value2 = 0
    Value3 = 0
    Value4 = 0
    NameStep = ""
    ActualLineInFile = 0

Move = [MoveParameter() for i in range(7500)]
IndexMoveParameter = 0
AmountMoveParameter = 0

# File variables
filename = ""   

# Com GUI write data
GUIWriteActualPos = [0, 0, 0, 0, 0, 0, 0, 0, # 16 Byte actual position axis
                     0, 0, 0, 0, 0, 0, 0, 0]

GUIWriteActualStep = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, # 100 Byte actual step
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

GUIWriteActualState = 0
GUIWriteDigitalState = 0
GUIReadManualCommands = 0
GUIReadSpindleSpeedManualMode = 0

WriteActualStep = 0
CompareMemoryWriteActualStep = 9999
StringWriteActualStep = ""

""" ############################################################################################ """
""" Function evaluate line from read file """
def evaluate_line(line):
    # Return variables
    GCode = 0
    Value0 = 99999999 # Variable not valid
    Value1 = 99999999 # Variable not valid
    Value2 = 99999999 # Variable not valid
    Value3 = 99999999 # Variable not valid
    Value4 = 99999999 # Variable not valid
    Valid = True
    
    # Local variables
    GCodeString = ""
    GCodeStringlen = 0
    GcodeIndex = 0
    Letter = ""
    Letter1 = ""
    ValueString = ""
    ValueNegative = False
    DotFound = False
    DecimalPlacesBeforeDot = 0
    DecimalPlacesAfterDot = 0
    TmpValue = 0
    G01Active = False
    G02Active = False
    G03Active = False
    LetterFound = False
    LetterGFound = False
    LetterXFound = False
    LetterYFound = False
    LetterZFound = False
    
    # Static local variable
    if 'CommentFound' not in evaluate_line.__dict__:
        evaluate_line.CommentFound = False

    if 'LastActiveCodeWasG00' not in evaluate_line.__dict__:
        evaluate_line.LastActiveCodeWasG00 = False

    if 'LastActiveCodeWasG01' not in evaluate_line.__dict__:
        evaluate_line.LastActiveCodeWasG01 = False

    if 'LastActiveCodeWasG02' not in evaluate_line.__dict__:
        evaluate_line.LastActiveCodeWasG02 = False

    if 'LastActiveCodeWasG03' not in evaluate_line.__dict__:
        evaluate_line.LastActiveCodeWasG03 = False
    
    # Transfer string and stringlen
    GCodeString = line
    GCodeStringlen = len(GCodeString)
    GCodeStringlen -= 1

    # Empty line
    if GCodeString == "" or GCodeStringlen == 0:
        Valid = False
        return(Valid, 0, 0, 0, 0, 0, 0)

    # Load first letter
    Letter = GCodeString[GcodeIndex]
    
    # Check if the first letter is "%" for header
    if Letter == "%":
        Valid = False
        return(Valid, 0, 0, 0, 0, 0, 0)

    # Check if the first letter is "(" for start comment
    if Letter == "(":
        evaluate_line.CommentFound = True # Set static variable commend found

        while GcodeIndex < GCodeStringlen:
            GcodeIndex += 1
            if GCodeString[GcodeIndex] == ")":
                evaluate_line.CommentFound = False # Comment is just in one line
    
        Valid = False
        return(Valid, 0, 0, 0, 0, 0, 0)   

    # Still a comment line, if comment line goes more than one line
    if evaluate_line.CommentFound == True and Letter != ")":
        Valid = False
        return(Valid, 0, 0, 0, 0, 0, 0)
        
    # End comment, if comment line goes more than one line
    if evaluate_line.CommentFound == True and Letter == ")":
        evaluate_line.CommentFound = False
        Valid = False
        return(Valid, 0, 0, 0, 0, 0, 0)       

    # G Code, M Code
    """
    Byte 1, G Code / M Code
    100 G00 Eilgang
    1 G01 Geradeninterpolation
    2 G02 Kreisinterpolation, im Uhrzeigersinn
    3 G03 Kreisinterpolation, gegen Uhrzeigersinn
    4 G04 Verweilzeit in Sekunden
    52  G52 Koordinatensystem Verschiebung
    54  G54 Nullpunktverschiebung auf Werkstücknullpunkt oder Programmnullpunkt
    74  G74 Referenzfahrt über Referenzfühler anfahren
    90  G90 Absolutmaßprogrammierung
    91  G91 Kettenmaßprogrammierung
    202 M02 Programmende
    203 M03 Spindle on, clockwise
    205 M05 Spindle off
    208 M08 Coolant on
    209 M09 Coolant off
    210 M10 Clamp on
    211 M11 Clamp off
    235 M35 Light on
    236 M36 Light off
    230 M30 Programmende mit Ruecksprung auf Programmanfang
    """

    while GcodeIndex < GCodeStringlen:
        
        """ ( found, comment at the end of the line """
        if Letter == "(":
            return(Valid, GCode, Value0, Value1, Value2, Value3, Value4)

        """ Key letter G found """
        if Letter == "G" or Letter == "g":
            # Initialize values
            ValueString = ""
            G01Active = False
            G02Active = False
            G03Active = False
            LetterFound = True
            GcodeIndex += 1
            LetterGFound = True
            evaluate_line.LastActiveCodeWasG00 = False
            evaluate_line.LastActiveCodeWasG01 = False
            evaluate_line.LastActiveCodeWasG02 = False
            evaluate_line.LastActiveCodeWasG03 = False
            
            # Build ValueString
            while (GCodeString[GcodeIndex].isnumeric() or GCodeString[GcodeIndex] == "." \
            or GCodeString[GcodeIndex] == ",") and GcodeIndex < GCodeStringlen:
                if GCodeString[GcodeIndex].isnumeric():
                    Letter1 = GCodeString[GcodeIndex]
                    ValueString = ValueString + Letter1
                    
                GcodeIndex += 1

            # Transfer value to GCode
            if ValueString.isnumeric:
                GCode = int(ValueString)
 
            # G00 Eilgang (100)
            if GCode == 0:
                GCode = 100
                Value3 = 0 # Reserve
                Value4 = 0 # Reserve
                evaluate_line.LastActiveCodeWasG00 = True
            
            # G01
            if GCode == 1:
                Value4 = 0 # Reserve
                G01Active = True
                evaluate_line.LastActiveCodeWasG01 = True
            
            # G02
            if GCode == 2:
                G02Active = True
                evaluate_line.LastActiveCodeWasG02 = True            
            
            # G03
            if GCode == 3:
                G03Active = True
                evaluate_line.LastActiveCodeWasG03 = True                     
            
            # G04
            if GCode == 4:
                Value1 = 0 # Reserve
                Value2 = 0 # Reserve
                Value3 = 0 # Reserve
                Value4 = 0 # Reserve

            # G52 or G54 or G74
            if GCode == 52 or GCode == 54 or GCode == 74:
                Value3 = 0 # Reserve
                Value4 = 0 # Reserve

            # G90 or G91
            if GCode == 90 or GCode == 91:
                Value0 = 0 # Reserve
                Value1 = 0 # Reserve
                Value2 = 0 # Reserve
                Value3 = 0 # Reserve
                Value4 = 0 # Reserve
            
        """ Key letter M found """
        if Letter == "M" or Letter == "m":
            # Initialize values
            LetterFound = True
            ValueString = ""
            GcodeIndex += 1

            # Build ValueString
            while (GCodeString[GcodeIndex].isnumeric() or GCodeString[GcodeIndex] == "." \
            or GCodeString[GcodeIndex] == ",") and GcodeIndex < GCodeStringlen:
                if GCodeString[GcodeIndex].isnumeric():
                    Letter1 = GCodeString[GcodeIndex]
                    ValueString = ValueString + Letter1
                    
                GcodeIndex += 1

            # Transfer value to GCode
            if ValueString.isnumeric:
                GCode = int(ValueString) + 200

            # M00 or M01 or M02 or M05 or M08 or M09 or M10 or M11 or M35 or M36 or M030
            if GCode == 200 or GCode == 201 or GCode == 202 or GCode == 205 \
            or GCode == 208 or GCode == 209 or GCode == 210 or GCode == 211 \
            or GCode == 235 or GCode == 236 or GCode == 230:
                Value0 = 0 # Reserve
                Value1 = 0 # Reserve
                Value2 = 0 # Reserve
                Value3 = 0 # Reserve
                Value4 = 0 # Reserve

            # M03
            if GCode == 203:
                Value1 = 0 # Reserve
                Value2 = 0 # Reserve
                Value3 = 0 # Reserve
                Value4 = 0 # Reserve

        """ Key letter X, Y, Z, I, J, F, S, P found """
        if Letter == "X" or Letter == "x" \
        or Letter == "Y" or Letter == "y" \
        or Letter == "Z" or Letter == "z" \
        or Letter == "I" or Letter == "i" \
        or Letter == "J" or Letter == "j" \
        or Letter == "F" or Letter == "f" \
        or Letter == "S" or Letter == "s" \
        or Letter == "P" or Letter == "p":
            # Initialize values
            LetterFound = True
            ValueString = ""
            ValueNegative = False
            DotFound = False
            DecimalPlacesBeforeDot = 0
            DecimalPlacesAfterDot = 0
            TmpValue = 0
            GcodeIndex += 1

            # Build ValueString
            while (GCodeString[GcodeIndex].isnumeric() or GCodeString[GcodeIndex] == "." \
            or GCodeString[GcodeIndex] == "," or GCodeString[GcodeIndex] == "-") \
            and DecimalPlacesAfterDot < 4 and GcodeIndex < GCodeStringlen:
                if GCodeString[GcodeIndex].isnumeric():
                    Letter1 = GCodeString[GcodeIndex]
                    ValueString = ValueString + Letter1
                    if DotFound == False:
                        DecimalPlacesBeforeDot += 1
                    else:
                        DecimalPlacesAfterDot +=1

                if GCodeString[GcodeIndex] == "-":
                    ValueNegative = True
                    
                if GCodeString[GcodeIndex] == "." or GCodeString[GcodeIndex] == ",":
                    DotFound = True

                GcodeIndex += 1

            # Calculate value decimal places
            if ValueString.isnumeric:
                TmpValue = int(ValueString)
            if DecimalPlacesAfterDot == 0:
                TmpValue *= 10000
            if DecimalPlacesAfterDot == 1:
                TmpValue *= 1000
            if DecimalPlacesAfterDot == 2:
                TmpValue *= 100
            if DecimalPlacesAfterDot == 3:
                TmpValue *= 10

            # Is value negative?
            if ValueNegative == True:
                TmpValue = TmpValue *-1
                
            # Transfer value
            if Letter == "X" or Letter == "x":
                Value0 = TmpValue
                LetterXFound = True
            if Letter == "Y" or Letter == "y":
                Value1 = TmpValue
                LetterYFound = True
            if Letter == "Z" or Letter == "z":
                Value2 = TmpValue
                LetterZFound = True
            if Letter == "I" or Letter == "i":
                Value2 = TmpValue       
            if Letter == "J" or Letter == "j":
                Value3 = TmpValue       
            if Letter == "F" or Letter == "f":
                # G01 is active
                if G01Active == True or \
                (G01Active == False and G02Active == False and G03Active == False and evaluate_line.LastActiveCodeWasG01 == True):
                    Value3 = TmpValue
                # G02 or G03 is active
                if G02Active == True or G03Active == True or \
                (G01Active == False and G02Active == False and G03Active == False and evaluate_line.LastActiveCodeWasG02 == True)  or \
                (G01Active == False and G02Active == False and G03Active == False and evaluate_line.LastActiveCodeWasG03 == True):
                    Value4 = TmpValue
            if Letter == "S" or Letter == "s": # M03, M04, S = Speed spindle
                Value0 = TmpValue       
            if Letter == "P" or Letter == "p": # G04 P = Dwell time
                Value0 = TmpValue       

        """ Load next letter """
        if LetterFound == False:
            GcodeIndex += 1
        LetterFound = False
        if GcodeIndex < GCodeStringlen:
            Letter = GCodeString[GcodeIndex]

    """ In Line no G-Code found, but X or Y or Z was found """
    if LetterGFound == False and (LetterXFound == True or LetterYFound == True or LetterZFound == True):
        if evaluate_line.LastActiveCodeWasG00 == True:
            GCode = 100
        if evaluate_line.LastActiveCodeWasG01 == True:
            GCode = 1
        if evaluate_line.LastActiveCodeWasG02 == True:
            GCode = 2
        if evaluate_line.LastActiveCodeWasG03 == True:
            GCode = 3

    return(Valid, GCode, Value0, Value1, Value2, Value3, Value4)

""" ############################################################################################ """
""" Function read file """
def read_file(name):
    # Local variables
    i = 0
    j = 0
    ReturnValue = 0
    Valid = False
    GCode = 0
    Value0 = 0
    Value1 = 0
    Value2 = 0
    Value3 = 0
    Value4 = 0
    
    # Global variables
    global Move

    # Open file readable only
    with open(name, 'r') as f:
        # Go to each line
        for line in f:
            # Call function "evaluate line from read file"
            (Valid, GCode, Value0, Value1, Value2, Value3, Value4) = evaluate_line(line)

            # Is the line in the file a valid G-Code            
            if Valid == True:
                # Copy G-Code to array, if Amount of lines < 7499
                if i < 7499:
                    Move[i].GCode = GCode
                    Move[i].Value0 = Value0
                    Move[i].Value1 = Value1
                    Move[i].Value2 = Value2
                    Move[i].Value3 = Value3
                    Move[i].Value4 = Value4
                    Move[i].NameStep = line
                    Move[i].ActualLineInFile = j
                    ReturnValue += 1
                    
                    """ Test Print G-Comand """
                    #print(i)
                    #print("Stepdata GCode %d Value0 %d Value1 %d Value2 %d Value3 %d Value4 %d" %(GCode, Value0, Value1, Value2, Value3, Value4))
                    
                # Amount of lines >= 7499
                else:
                    ReturnValue = 999999

                # Index move parameter + 1
                i += 1
                
            
            # Actual line in file + 1
            j += 1

    # Close file
    f.close()

    # If last line GCode = 203 -> GCode 230
    # If last line GCode = 200 -> GCode 202
    # This is the case, if after M030 no comment or no next line is in the file
    # This is the case, if after M02 no comment or no next line is in the file
    if Move[i-1].GCode != 230:
        Move[i-1].GCode = 230

    return(ReturnValue)

""" ############################################################################################ """
""" Function read bit motor """
# Converts recieved byte to bits
def read_bit_motor(data):
    TmpStateData = data & 0b00000001
    Bit0 = False
    if TmpStateData == 1:
        Bit0 = True
    
    TmpStateData = data & 0b00000010
    Bit1 = False
    if TmpStateData == 2:
        Bit1 = True
        
    TmpStateData = data & 0b00000100
    Bit2 = False
    if TmpStateData == 4:
        Bit2 = True
        
    TmpStateData = data & 0b00001000
    Bit3 = False
    if TmpStateData == 8:
        Bit3 = True
        
    TmpStateData = data & 0b00010000
    Bit4 = False
    if TmpStateData == 16:
        Bit4 = True
    
    TmpStateData = data & 0b00100000
    Bit5 = False
    if TmpStateData == 32:
        Bit5 = True
        
    TmpStateData = data & 0b01000000
    Bit6 = False
    if TmpStateData == 64:
        Bit6 = True
    
    TmpStateData = data & 0b10000000
    Bit7 = False
    if TmpStateData == 128:
        Bit7 = True        

    return(Bit0, Bit1, Bit2, Bit3, \
           Bit4, Bit5, Bit6, Bit7)

""" ############################################################################################ """
""" Function hexadecimal to decimal """
def hexadecimalToDecimal(hexval): 
      
    # Finding length 
    length = len(hexval) 
      
    # Initialize base value to 1, 
    # i.e. 16*0 
    base = 1
    dec_val = 0
      
    # Extracting characters as digits 
    # from last character 
    for i in range(length - 1, -1, -1): 
          
        # If character lies in '0'-'9',  
        # converting it to integral 0-9   
        # by subtracting 48 from ASCII value 
        if hexval[i] >= '0' and hexval[i] <= '9': 
            dec_val += (ord(hexval[i]) - 48) * base 
              
            # Incrementing base by power 
            base = base * 16
          
        # If character lies in 'A'-'F',converting 
        # it to integral 10-15 by subtracting 55 
        # from ASCII value 
        elif hexval[i] >= 'A' and hexval[i] <= 'F': 
            dec_val += (ord(hexval[i]) - 55) * base 
              
            # Incrementing base by power 
            base = base * 16
              
    return dec_val 

""" ############################################################################################ """
""" Function decimal to hexadecimal to decimal covering positive and negative numbers """
# 1. Convert decimal value to hex                        -->> dec999 -> hex00 00 03 E7
# 2. Two hex values to decimal (One Byte)                -->> hex00->dec0, hex00->dec0, hex03->dec3, hexE7->dec231
# 3. Returns 4 bytes, containing always two hex values   -->> dec 0, 0, 3, 231
def decToHexToDec(decval) :  
    # Local variables
    Byte0 = 0 # Return value decimal byte 0
    Byte1 = 0 # Return value decimal byte 1
    Byte2 = 0 # Return value decimal byte 2
    Byte3 = 0 # Return value decimal byte 3
    tmpString = ('00') # Temporary char array, length 2 char
    ValuePositive = True

    # Local variables positive number
    hexaDeciNum = ['0'] * 10 # char array to store hexadecimal number, length 10 char
    temp = 0 # temporary variable to store remainder 
    i = 0 # counter for hexadecimal number array 

    # Local variables negative number
    m = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F']
    n = 0
    res = "" # string to store hexadecimal number

    # Number = 0, decval==0
    if(not decval): 
        return(0, 0, 0, 0) # Return values, 4 byte decimal
  
    # Positive number, decval>0
    if(decval > 0):
        while(decval != 0): 
            temp = decval % 16 # storing remainder in temp variable
            # check if temp < 10 
            if(temp < 10): 
                hexaDeciNum[i] = chr(temp + 48)
                i = i + 1
            else: 
                hexaDeciNum[i] = chr(temp + 55)
                i = i + 1
            decval = int(decval / 16)
      
    # Negative number, decval<0
    else:
        ValuePositive = False
        n = decval + 2**32
        while(n): 
            res = m[n % 16] + res
            n //= 16

    if(ValuePositive == True):
        tmpString = hexaDeciNum[1] + hexaDeciNum[0]
        Byte0 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal
        tmpString = hexaDeciNum[3] + hexaDeciNum[2]
        Byte1 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal
        tmpString = hexaDeciNum[5] + hexaDeciNum[4]
        Byte2 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal
        tmpString = hexaDeciNum[7] + hexaDeciNum[6]
        Byte3 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal

    else:
        tmpString = res[6] + res[7]
        Byte0 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal
        tmpString = res[4] + res[5]
        Byte1 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal
        tmpString = res[2] + res[3]
        Byte2 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal
        tmpString = res[0] + res[1]
        Byte3 = hexadecimalToDecimal(tmpString) # Call function hexadecimal to decimal

    return(Byte0, Byte1, Byte2, Byte3) # Return values, 4 byte decimal

""" ############################################################################################ """
""" Funktion main """
def main(ComGUIArrayInt, ComGUIArrayIntStringActualStep, ComGUIArrayIntStringActualFile, lock):

    """ ############################################################################################ """
    """ Global variables """
    # Read data I2C
    global OperateOn # Bit data Byte 0
    global Reserve0_1
    global Reserve0_2
    global Reserve0_3
    global Reserve0_4
    global Reserve0_5
    global Reserve0_6
    global Reserve0_7
    global CommandDone # Bit data Byte 1
    global ReadyForCommand
    global ProgramDone
    global Reserve1_3
    global Reserve1_4
    global Reserve1_5
    global Reserve1_6
    global Reserve1_7
    global ReferenceSensorX # Bit data Byte 2
    global ReferenceSensorY
    global ReferenceSensorZ
    global EmergencyStopButton
    global SpindleOn
    global CoolantOn
    global ClampOn
    global LightOn
    global ToolLengthSensor # Bit data Byte 3
    global MeasureToolLengthDone
    global Reserve3_2
    global Reserve3_3
    global Reserve3_4
    global Reserve3_5
    global Reserve3_6
    global Reserve3_7
    global ReadDataMotor
    global CounterReadErrorI2C
    global ReadDataAreValid
    global ChecksumRead

    # Write data I2C
    global WriteByte0BitData
    global WriteByte1BitData
    global WriteByte2BitData
    global WriteByte3BitData
    global WriteByte4 # Spindle speed manual mode
    global WriteByte5 # Spindle speed manual mode
    global WriteByte6GCodeMCode
    global WriteByte7
    global WriteByte8
    global WriteByte9
    global WriteByte10
    global WriteByte11
    global WriteByte12
    global WriteByte13
    global WriteByte14
    global WriteByte15
    global WriteByte16
    global WriteByte17
    global WriteByte18
    global WriteByte19
    global WriteByte20
    global WriteByte21
    global WriteByte22
    global WriteByte23
    global WriteByte24
    global WriteByte25
    global WriteByte26
    global WriteByte27Checksum
    global WriteDataMotor
    global CounterWriteErrorI2C
    global WriteDataAreValid

    # I2C statistic data
    global CounterI2CReciveGood
    global CounterI2CReciveChecksumError
    global CounterI2CReciveHardwareError
    global CounterI2CSendGood
    global CounterI2CSendHardwareError
    
    # Common Variables
    global ActualStepProgram
    global DecValueByte
    global InchingActive
    global ManualMode
    global MeasureToolLength

    # Stepvariables
    global CaseIdle
    global CaseTransferFilename
    global CaseReadFile
    global CaseTransferFile
    global CaseMotHandshakeCommandDone
    global CaseMotOperateOn
    global CaseMotOperateOff
    global CaseMotInching
    global CaseMeasureToolLength
    global StepNoMotor
    
    # Print actual step
    global CompareMemoryActualStep

    # Class move parameter
    global Move
    global IndexMoveParameter
    global AmountMoveParameter
    
    # File variables
    global filename

    # Com GUI write data
    global GUIWriteActualPos
    global GUIWriteActualStep
    global GUIWriteActualState
    global GUIWriteDigitalState
    global GUIReadManualCommands
    global GUIReadSpindleSpeedManualMode
    global WriteActualStep
    global CompareMemoryWriteActualStep

    """ ######################################################################################## """
    """ Start loop forever """
    while True:

        """ ######################################################################################## """
        """ Read data GUI -> Mill, Shared memory, each program scan """
        with lock:
            # GUI -> Mill, Shared memory
            # ComGUIArrayInt[1] GUI -> Mill ManualCommands. Bit 0 = Manual command Spindle on
            #                                               Bit 1 = Manual command Coolant on
            #                                               Bit 2 = Manual command Clamp on
            #                                               Bit 3 = Manual command Light on        
            #                                               Bit 4 = Manual command X+
            #                                               Bit 5 = Manual command X-
            #                                               Bit 6 = Manual command Y+
            #                                               Bit 7 = Manual command Y-
            #                                               Bit 8 = Manual command Z+
            #                                               Bit 9 = Manual command Z-
            #                                               Bit 10 = ClearDiagnosticCounterI2C
            #                                               Bit 11 = Manual mode selected
            #                                               Bit 12 = Measure tool length
            # ComGUIArrayInt[2] GUI -> Mill Spindle speed in manual mode
            GUIReadManualCommands = ComGUIArrayInt[1] # Bit 0 = Manual command Spindle on, ...
            GUIReadSpindleSpeedManualMode = ComGUIArrayInt[2] # Spindle speed manual mode

        # Mill -> Teensy I2C, Byte 2
        # Bit 0 = Manual command Spindle on
        # Bit 1 = Manual command Coolant on
        # Bit 2 = Manual command Clamp on
        # Bit 3 = Manual command Light on
        # Bit 4 = Manual command X+
        # Bit 5 = Manual command X-
        # Bit 6 = Manual command Y+
        # Bit 7 = Manual command Y-

        # Mill ->  Teensy I2C, Byte 3
        # Bit 0 = Manual command Z+
        # Bit 1 = Manual command Z-
        # Bit 2 = ClearDiagnosticCounterI2C
        # Bit 3 = Measure tool length

        InchingActive = False
        ManualMode = False
        MeasureToolLength = False
        
        # To Teensy I2C, Byte 2
        if GUIReadManualCommands & 1 != 0: # Check bit 0 Spindle on
            WriteByte2BitData = WriteByte2BitData | 0b00000001 # Set bit 0 Spindle on
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b11111110 # Reset bit 0 Spindle on
            
        if GUIReadManualCommands & 2 != 0: # Check bit 1 Coolant on
            WriteByte2BitData = WriteByte2BitData | 0b00000010 # Set bit 1 Coolant on
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b11111101 # Reset bit 1 Coolant on        
        
        if GUIReadManualCommands & 4 != 0: # Check bit 2 Clamp on
            WriteByte2BitData = WriteByte2BitData | 0b00000100 # Set bit 2 Clamp on
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b11111011 # Reset bit 2 Clamp on   
        
        if GUIReadManualCommands & 8 != 0: # Check bit 3 Light on
            WriteByte2BitData = WriteByte2BitData | 0b00001000 # Set bit 3 Light on
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b11110111 # Reset bit 3 Light on

        if GUIReadManualCommands & 16 != 0: # Check bit 4 X+
            WriteByte2BitData = WriteByte2BitData | 0b00010000 # Set bit 4 X+
            InchingActive = True
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b11101111 # Reset bit 4 X+
            
        if GUIReadManualCommands & 32 != 0: # Check bit 5 X-
            WriteByte2BitData = WriteByte2BitData | 0b00100000 # Set bit 5 X-
            InchingActive = True
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b11011111 # Reset bit 5 X-

        if GUIReadManualCommands & 64 != 0: # Check bit 6 Y+
            WriteByte2BitData = WriteByte2BitData | 0b01000000 # Set bit 6 Y+
            InchingActive = True
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b10111111 # Reset bit 6 Y+

        if GUIReadManualCommands & 128 != 0: # Check bit 7 Y-
            WriteByte2BitData = WriteByte2BitData | 0b10000000 # Set bit 7 Y-
            InchingActive = True
            ManualMode = True
        else:
            WriteByte2BitData = WriteByte2BitData & 0b01111111 # Reset bit 7 Y-

        # To Teensy I2C, Byte 3
        if GUIReadManualCommands & 256 != 0: # Check bit 8 Z+
            WriteByte3BitData = WriteByte3BitData | 0b00000001 # Set bit 0 Z+
            InchingActive = True
            ManualMode = True
        else:
            WriteByte3BitData = WriteByte3BitData & 0b11111110 # Reset bit 0 Z+

        if GUIReadManualCommands & 512 != 0: # Check bit 9 Z-
            WriteByte3BitData = WriteByte3BitData | 0b00000010 # Set bit 1 Z-
            InchingActive = True
            ManualMode = True
        else:
            WriteByte3BitData = WriteByte3BitData & 0b11111101 # Reset bit 1 Z-

        if GUIReadManualCommands & 1024 != 0: # Check bit 10 ClearDiagnosticCounterI2C
            WriteByte3BitData = WriteByte3BitData | 0b00000100 # Set bit 2 ClearDiagnosticCounterI2C
        else:
            WriteByte3BitData = WriteByte3BitData & 0b11111011 # Reset bit 2 ClearDiagnosticCounterI2C

        # Bit 10 = ClearDiagnosticCounterI2C
        if GUIReadManualCommands & 1024 != 0: # Check bit 10 ClearDiagnosticCounterI2C
            CounterI2CReciveGood = 0
            CounterI2CReciveChecksumError = 0
            CounterI2CReciveHardwareError = 0
            CounterI2CSendGood = 0
            CounterI2CSendHardwareError = 0
            
        # Bit 11 = Manual mode selected
        if GUIReadManualCommands & 2048 != 0: # Check bit 11 Manual mode selected
            ManualMode = True

        # Bit 12 = Measure tool length
        if GUIReadManualCommands & 4096 != 0: # Check bit 12 Measure tool length
            WriteByte3BitData = WriteByte3BitData | 0b00001000 # Set bit 3 Measure tool length
            MeasureToolLength = True
            ManualMode = True
        else:
            WriteByte3BitData = WriteByte3BitData & 0b11110111 # Reset bit 3 Measure tool length

        """ ######################################################################################## """
        """ Read data Teensy, I2C, each program scan """
        ReadDataAreValid = False
        CounterReadErrorI2C = 0
        while CounterReadErrorI2C <= 4 and ReadDataAreValid == False: # Try 5 times to read I2C data
            try:
                """ Read byte 0..22 (23 bytes, Startregister = 0) from I2C_ADDR_Teensy """
                ReadDataMotor = bus_3.read_i2c_block_data(I2C_ADDR_Teensy, 0x00, 23) 
    
                """ Checksum """
                ChecksumRead = ReadDataMotor[0] ^ ReadDataMotor[1] ^ ReadDataMotor[2] ^ ReadDataMotor[3] \
                                ^ ReadDataMotor[4] ^ ReadDataMotor[5] ^ ReadDataMotor[6] ^ ReadDataMotor[7] \
                                ^ ReadDataMotor[8] ^ ReadDataMotor[9] ^ ReadDataMotor[10] ^ ReadDataMotor[11] \
                                ^ ReadDataMotor[12] ^ ReadDataMotor[13] ^ ReadDataMotor[14] ^ ReadDataMotor[15] \
                                ^ ReadDataMotor[16] ^ ReadDataMotor[17] ^ ReadDataMotor[18] ^ ReadDataMotor[19] \
                                ^ ReadDataMotor[20] ^ ReadDataMotor[21]
                                
    
                if ChecksumRead == ReadDataMotor[22]:
                    
                    """ Call function "read_bit_motor" converts recieved byte to bits """
                    (OperateOn, Reserve0_1, Reserve0_2, Reserve0_3, \
                     Reserve0_4, Reserve0_5, Reserve0_6, Reserve0_7) \
                     = read_bit_motor(ReadDataMotor[0])
                     
                    (CommandDone, ReadyForCommand, ProgramDone, Reserve1_3, \
                     Reserve1_4, Reserve1_5, Reserve1_6, Reserve1_7) \
                     = read_bit_motor(ReadDataMotor[1])
                     
                    (ReferenceSensorX, ReferenceSensorY, ReferenceSensorZ, EmergencyStopButton, \
                     SpindleOn, CoolantOn, ClampOn, LightOn) \
                     = read_bit_motor(ReadDataMotor[2])                     
                     
                    (ToolLengthSensor, MeasureToolLengthDone, Reserve3_2, Reserve3_3, \
                     Reserve3_4, Reserve3_5, Reserve3_6, Reserve3_7) \
                     = read_bit_motor(ReadDataMotor[3])                     

                    """ Transfer actual positions """
                    GUIWriteActualPos[0] = ReadDataMotor[4]
                    GUIWriteActualPos[1] = ReadDataMotor[5]
                    GUIWriteActualPos[2] = ReadDataMotor[6]
                    GUIWriteActualPos[3] = ReadDataMotor[7]
                    GUIWriteActualPos[4] = ReadDataMotor[8]
                    GUIWriteActualPos[5] = ReadDataMotor[9]
                    GUIWriteActualPos[6] = ReadDataMotor[10]
                    GUIWriteActualPos[7] = ReadDataMotor[11]
                    GUIWriteActualPos[8] = ReadDataMotor[12]
                    GUIWriteActualPos[9] = ReadDataMotor[13]
                    GUIWriteActualPos[10] = ReadDataMotor[14]
                    GUIWriteActualPos[11] = ReadDataMotor[15]
                    GUIWriteActualPos[12] = ReadDataMotor[16]
                    GUIWriteActualPos[13] = ReadDataMotor[17]
                    GUIWriteActualPos[14] = ReadDataMotor[18]
                    GUIWriteActualPos[15] = ReadDataMotor[19]

                    """ Transfer actual step program """
                    ActualStepProgram = ReadDataMotor[20] << 8
                    ActualStepProgram = ActualStepProgram | ReadDataMotor[21]
    
                    ReadDataAreValid = True
                    CounterI2CReciveGood +=1
    
                else:
                    CounterI2CReciveChecksumError += 1
    
            except IOError:
                CounterReadErrorI2C = CounterReadErrorI2C + 1
                CounterI2CReciveHardwareError +=1

        """ ######################################################################################## """
        """ Step Idle """
        if StepNoMotor == CaseIdle:
            # Transfer actual step
            if ManualMode == True:
                StringWriteActualStep = ("Manual mode")
                WriteActualStep = 9997
            else:
                StringWriteActualStep = ("Idle")
                WriteActualStep = 9998

            # Write state to GUI
            GUIWriteActualState = 0 # State. 0=Idle, 1=Program active, 2=Read file active

            # Command start read file or command start program
            with lock:
                if ComGUIArrayInt[0] == 1: # Command Read file
                    ComGUIArrayInt[0] = 0
                    StepNoMotor = CaseTransferFilename
                    
                if ComGUIArrayInt[0] == 2: # Command Start program
                    ComGUIArrayInt[0] = 0
                    CompareMemoryActualStep = 9999
                    CompareMemoryWriteActualStep = 9999
                    StepNoMotor = CaseMotOperateOn

            # Start inching
            if InchingActive == True:
                StepNoMotor = CaseMotInching
                
            # Start measure tool length
            if MeasureToolLength == True:
                StepNoMotor = CaseMeasureToolLength

        """ ######################################################################################## """
        """ Step Transfer filename """
        if StepNoMotor == CaseTransferFilename:

            # Write state to GUI
            GUIWriteActualState = 2 # State. 0=Idle, 1=Program active, 2=Read file active
            
            # Clear filename
            filename = ""
            
            # Read shared memory filename
            with lock:
                # Actual step
                for i in range(1, ComGUIArrayIntStringActualFile[0]+1):
                    TmpValue = ComGUIArrayIntStringActualFile[i]
                    filename = filename + chr(TmpValue)

            StepNoMotor = CaseReadFile
    
        """ ######################################################################################## """
        """ Step Read file """
        if StepNoMotor == CaseReadFile:
            # Transfer actual step
            StringWriteActualStep = ("Read file")
            WriteActualStep = 9997

            # Call function read file
            AmountMoveParameter = read_file(filename)
            IndexMoveParameter = 0
            StepNoMotor = CaseTransferFile

        """ ######################################################################################## """
        """ Step Transfer stepdata to teensy """
        if StepNoMotor == CaseTransferFile:
            if ReadyForCommand == True:
                # Activate command
                WriteByte1BitData = WriteByte1BitData | 0b00000001 # Set Byte 1 bit 0 ActivateCommand
                # G-Code, M-Code
                WriteByte6GCodeMCode = Move[IndexMoveParameter].GCode
                # Value 0
                DecValueByte[0], DecValueByte[1], DecValueByte[2], DecValueByte[3] = decToHexToDec(Move[IndexMoveParameter].Value0)
                WriteByte10 = DecValueByte[0]
                WriteByte9 = DecValueByte[1]
                WriteByte8 = DecValueByte[2]
                WriteByte7 = DecValueByte[3]
                # Value 1
                DecValueByte[0], DecValueByte[1], DecValueByte[2], DecValueByte[3] = decToHexToDec(Move[IndexMoveParameter].Value1)
                WriteByte14 = DecValueByte[0]
                WriteByte13 = DecValueByte[1]
                WriteByte12 = DecValueByte[2]
                WriteByte11 = DecValueByte[3]
                # Value 2
                DecValueByte[0], DecValueByte[1], DecValueByte[2], DecValueByte[3] = decToHexToDec(Move[IndexMoveParameter].Value2)
                WriteByte18 = DecValueByte[0]
                WriteByte17 = DecValueByte[1]
                WriteByte16 = DecValueByte[2]
                WriteByte15 = DecValueByte[3]
                # Value 3
                DecValueByte[0], DecValueByte[1], DecValueByte[2], DecValueByte[3] = decToHexToDec(Move[IndexMoveParameter].Value3)
                WriteByte22 = DecValueByte[0]
                WriteByte21 = DecValueByte[1]
                WriteByte20 = DecValueByte[2]
                WriteByte19 = DecValueByte[3]
                # Value 4
                DecValueByte[0], DecValueByte[1], DecValueByte[2], DecValueByte[3] = decToHexToDec(Move[IndexMoveParameter].Value4)
                WriteByte26 = DecValueByte[0]
                WriteByte25 = DecValueByte[1]
                WriteByte24 = DecValueByte[2]
                WriteByte23 = DecValueByte[3]

            if CommandDone == True and ReadyForCommand == False:
                # Transfer actual step
                StringWriteActualStep = ("Load step: %d Of %d: %s" % (IndexMoveParameter +1, AmountMoveParameter, Move[IndexMoveParameter].NameStep))
                WriteActualStep = IndexMoveParameter +1

                IndexMoveParameter += 1
                StepNoMotor = CaseMotHandshakeCommandDone

        """ ######################################################################################## """
        """ Handshake command done """
        if StepNoMotor == CaseMotHandshakeCommandDone:
            WriteByte1BitData = WriteByte1BitData & 0b11111110 # Reset Byte 1 bit 0 ActivateCommand

            if CommandDone == False and ReadyForCommand == True:
                if IndexMoveParameter < AmountMoveParameter:
                    StepNoMotor = CaseTransferFile
                else:
                    StepNoMotor = CaseIdle

        """ ######################################################################################## """
        """ Operate on """
        if StepNoMotor == CaseMotOperateOn:
            # Switch operate on
            WriteByte0BitData = WriteByte0BitData | 0b00000001 # Set Byte 0 bit 0 Operate on
            # Write state to GUI
            GUIWriteActualState = 1 # State. 0=Idle, 1=Program active, 2=Read file active
            # Print actual execute step No.
            if ActualStepProgram != CompareMemoryActualStep:
                if ActualStepProgram >= 0 and ActualStepProgram <= 7499:
                    # Transfer actual step
                    StringWriteActualStep = ("Execute step %d Of %d: %s" %(ActualStepProgram +1, AmountMoveParameter, Move[ActualStepProgram].NameStep))
                    WriteActualStep = ActualStepProgram
                    CompareMemoryActualStep = ActualStepProgram
            # Program done
            if ProgramDone == True:
                StepNoMotor = CaseMotOperateOff

            # Stop program
            with lock:
                if ComGUIArrayInt[0] == 3: # Command stop program
                    ComGUIArrayInt[0] = 0
                    StepNoMotor = CaseMotOperateOff

        """ ######################################################################################## """
        """ Operate off, go back to idle """
        if StepNoMotor == CaseMotOperateOff:
            WriteByte0BitData = WriteByte0BitData & 0b11111110 # Reset Byte 0 bit 0 Operate on
            if OperateOn == False:
                StepNoMotor = CaseIdle

        """ ######################################################################################## """
        """ Inching """
        if StepNoMotor == CaseMotInching:
            # Switch operate on
            WriteByte0BitData = WriteByte0BitData | 0b00000001 # Set Byte 0 bit 0 Operate on
            # Transfer actual step
            StringWriteActualStep = ("Inching")
            WriteActualStep = 9996
            
            if InchingActive == False:
                StepNoMotor = CaseMotOperateOff

        """ ######################################################################################## """
        """ Measure tool length """
        if StepNoMotor == CaseMeasureToolLength:
            # Switch operate on
            WriteByte0BitData = WriteByte0BitData | 0b00000001 # Set Byte 0 bit 0 Operate on
            # Transfer actual step
            StringWriteActualStep = ("Measure tool length")
            WriteActualStep = 9995
            
            if MeasureToolLength == False:
                StepNoMotor = CaseMotOperateOff

        """ ######################################################################################## """
        """ Transfer spindle speed manual mode """
        WriteByte4 = GUIReadSpindleSpeedManualMode & 0x00FF
        WriteByte5 = (GUIReadSpindleSpeedManualMode & 0xFF00) >> 8

        """ ######################################################################################## """
        """ Create checksum write data """
        WriteByte27Checksum = WriteByte0BitData ^ WriteByte1BitData  ^ WriteByte2BitData ^ WriteByte3BitData \
                            ^ WriteByte4 ^ WriteByte5 \
                            ^ WriteByte6GCodeMCode ^ WriteByte7 ^ WriteByte8 ^ WriteByte9 \
                            ^ WriteByte10 ^ WriteByte11 ^ WriteByte12 ^ WriteByte13 \
                            ^ WriteByte14 ^ WriteByte15 ^ WriteByte16 ^ WriteByte17 \
                            ^ WriteByte18 ^ WriteByte19 ^ WriteByte20 ^ WriteByte21 \
                            ^ WriteByte22 ^ WriteByte23 ^ WriteByte24 ^ WriteByte25 ^ WriteByte26
            
        """ ######################################################################################## """
        """ Write data Teensy, I2C, each program scan """
        WriteDataMotor = [WriteByte0BitData, WriteByte1BitData, WriteByte2BitData, WriteByte3BitData,
                          WriteByte4, WriteByte5,
                          WriteByte6GCodeMCode, WriteByte7, WriteByte8, WriteByte9,
                          WriteByte10, WriteByte11, WriteByte12, WriteByte13,
                          WriteByte14, WriteByte15, WriteByte16, WriteByte17,
                          WriteByte18, WriteByte19, WriteByte20, WriteByte21,
                          WriteByte22, WriteByte23, WriteByte24, WriteByte25,
                          WriteByte26, WriteByte27Checksum]

        WriteDataAreValid = False
        CounterWriteErrorI2C = 0
        while CounterWriteErrorI2C <= 4 and WriteDataAreValid == False: # Try 5 times to write I2C data
            try: 
                """ Write byte 0..25 (26 bytes, Startregister = 1) from I2C_ADDR_Teensy """
                bus_3.write_i2c_block_data(I2C_ADDR_Teensy, 0x01, WriteDataMotor)
                WriteDataAreValid = True
                CounterI2CSendGood +=1

            except IOError:
                CounterWriteErrorI2C = CounterWriteErrorI2C + 1
                CounterI2CSendHardwareError += 1

        """ ######################################################################################## """
        """ Actualize string actual step """
        # Value 0 = Stringlen, Value 1..99 = Stringdata
        if WriteActualStep != CompareMemoryWriteActualStep:
            CompareMemoryWriteActualStep = WriteActualStep
            
            # Transfer stringlen, max 98
            GCodeStringlen = len(StringWriteActualStep)
            if GCodeStringlen > 98:
                GCodeStringlen = 98
            GUIWriteActualStep[0] = GCodeStringlen
            
             # Actual step, ASCII characters to INT values
            for i in range(GCodeStringlen):
                Letter = StringWriteActualStep[i]
                GUIWriteActualStep[i+1] = ord(Letter)

        """ ######################################################################################## """
        """ Actualize digital states """
        # Bit 0 = Reference sensor X
        # Bit 1 = Reference sensor Y
        # Bit 2 = Reference sensor Z
        # Bit 3 = Emergency stop button
        # Bit 4 = Spindle on
        # Bit 5 = Coolant on
        # Bit 6 = Clamp on
        # Bit 7 = Light on
        # Bit 8 = Tool length sensor
        # Bit 9 = Measure tool length done
        
        GUIWriteDigitalState = 0

        if ReferenceSensorX == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 1
            
        if ReferenceSensorY == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 2

        if ReferenceSensorZ == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 4

        if EmergencyStopButton == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 8

        if SpindleOn == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 16

        if CoolantOn == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 32

        if ClampOn == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 64

        if LightOn == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 128

        if ToolLengthSensor == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 256
            
        if MeasureToolLengthDone == True:
            GUIWriteDigitalState = GUIWriteDigitalState | 512
            

        """ ######################################################################################## """
        """ Write data Mill -> GUI, Shared memory, each program scan """
        with lock:
            
            # Mill -> GUI Actual line number in file
            if ActualStepProgram >= 0 and ActualStepProgram <= 7499:
                ComGUIArrayInt[8] = Move[ActualStepProgram].ActualLineInFile
                
            # Mill -> GUI Write state to GUI
            ComGUIArrayInt[9] = GUIWriteActualState # State. 0=Idle, 1=Program active, 2=Read file active

            # Mill -> GUI Write digital states to GUI
            ComGUIArrayInt[10] = GUIWriteDigitalState # Bit 0 = Reference sensor X, ...
            
            # Mill -> GUI Diagnostic data
            ComGUIArrayInt[11] = CounterI2CReciveGood # Diagnostic I2C Recive good
            ComGUIArrayInt[12] = CounterI2CReciveChecksumError # Diagnostic I2C Recive checksum error
            ComGUIArrayInt[13] = CounterI2CReciveHardwareError # Diagnostic I2C Recive hardware error
            ComGUIArrayInt[14] = CounterI2CSendGood # Diagnostic I2C Send good
            ComGUIArrayInt[15] = CounterI2CSendHardwareError # Diagnostic I2C Send hardware error

            # Mill -> GUI Actual position, INT values
            ComGUIArrayInt[16] = GUIWriteActualPos[0]
            ComGUIArrayInt[17] = GUIWriteActualPos[1]
            ComGUIArrayInt[18] = GUIWriteActualPos[2]
            ComGUIArrayInt[19] = GUIWriteActualPos[3]
            ComGUIArrayInt[20] = GUIWriteActualPos[4]
            ComGUIArrayInt[21] = GUIWriteActualPos[5]
            ComGUIArrayInt[22] = GUIWriteActualPos[6]
            ComGUIArrayInt[23] = GUIWriteActualPos[7]
            ComGUIArrayInt[24] = GUIWriteActualPos[8]
            ComGUIArrayInt[25] = GUIWriteActualPos[9]
            ComGUIArrayInt[26] = GUIWriteActualPos[10]
            ComGUIArrayInt[27] = GUIWriteActualPos[11]
            ComGUIArrayInt[28] = GUIWriteActualPos[12]
            ComGUIArrayInt[29] = GUIWriteActualPos[13]
            ComGUIArrayInt[30] = GUIWriteActualPos[14]
            ComGUIArrayInt[31] = GUIWriteActualPos[15]

            # Mill -> GUI Actual step. Value 0 = Stringlen, Value 1..99 = Stringdata
            for i in range(GCodeStringlen + 1):
                ComGUIArrayIntStringActualStep[i] = GUIWriteActualStep[i]

    """ ############################################################################################ """
    """ End loop forever """









