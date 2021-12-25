""" ############################################################################################ """
""" ############################################################################################ """
""" GUI CNC mill with Teensy4.0 """
""" V1_00, 2021-12-24, are """
""" ############################################################################################ """
""" ############################################################################################ """

"""
##### GUI <--> Mill, Shared memory
PythonProgramGUIPortalfraese.py <--> PythonProgrammPortalfraese.py
ComGUIArrayInt = multiprocessing.Array('i', range(32))
ComGUIArrayIntStringActualStep = multiprocessing.Array('i', range(100))
ComGUIArrayIntStringActualFile = multiprocessing.Array('i', range(256))

##### ComGUIArrayInt
ComGUIArrayInt[0] GUI -> Mill Command. 1=read file, 2=start program, 3=stop program. Mill -> GUI 0 = command read
ComGUIArrayInt[1] GUI -> Mill ManualCommands.   Bit 0 = Manual command Spindle on
                                                Bit 1 = Manual command Coolant on
                                                Bit 2 = Manual command Clamp on
                                                Bit 3 = Manual command Light on
                                                Bit 4 = Manual command X+
                                                Bit 5 = Manual command X-
                                                Bit 6 = Manual command Y+
                                                Bit 7 = Manual command Y-
                                                Bit 8 = Manual command Z+
                                                Bit 9 = Manual command Z-
                                                Bit 10 = ClearDiagnosticCounterI2C
                                                Bit 11 = Manual mode selected
                                                Bit 12 = Measure tool length
ComGUIArrayInt[2] GUI -> Mill Spindle speed in manual mode
ComGUIArrayInt[3]
ComGUIArrayInt[4]
ComGUIArrayInt[5]
ComGUIArrayInt[6]
ComGUIArrayInt[7]
ComGUIArrayInt[8]  Mill -> GUI Actual line number in file
ComGUIArrayInt[9]  Mill -> GUI State. 0=Idle, 1=Program active, 2=Read file active
ComGUIArrayInt[10] Mill -> GUI Digital states.  Bit 0 = Reference sensor X
                                                Bit 1 = Reference sensor Y
                                                Bit 2 = Reference sensor Z
                                                Bit 3 = Emergency stop button
                                                Bit 4 = Spindle on
                                                Bit 5 = Coolant on
                                                Bit 6 = Clamp on
                                                Bit 7 = Light on
                                                Bit 8 = Tool length sensor
                                                Bit 9 = Measure tool length done
ComGUIArrayInt[11] Mill -> GUI Diagnostic I2C Recive good
ComGUIArrayInt[12] Mill -> GUI Diagnostic I2C Recive checksum error
ComGUIArrayInt[13] Mill -> GUI Diagnostic I2C Recive hardware error
ComGUIArrayInt[14] Mill -> GUI Diagnostic I2C Send good
ComGUIArrayInt[15] Mill -> GUI Diagnostic I2C Send hardware error
ComGUIArrayInt[16] Mill -> GUI actual position X HighHigh
ComGUIArrayInt[17] Mill -> GUI actual position X High
ComGUIArrayInt[18] Mill -> GUI actual position X Low
ComGUIArrayInt[19] Mill -> GUI actual position X LowLow
ComGUIArrayInt[20] Mill -> GUI actual position Y HighHigh
ComGUIArrayInt[21] Mill -> GUI actual position Y High
ComGUIArrayInt[22] Mill -> GUI actual position Y Low
ComGUIArrayInt[23] Mill -> GUI actual position Y LowLow
ComGUIArrayInt[24] Mill -> GUI actual position Z HighHigh
ComGUIArrayInt[25] Mill -> GUI actual position Z High
ComGUIArrayInt[26] Mill -> GUI actual position Z Low
ComGUIArrayInt[27] Mill -> GUI actual position Z LowLow
ComGUIArrayInt[28] Mill -> GUI actual position Reserve
ComGUIArrayInt[29] Mill -> GUI actual position Reserve
ComGUIArrayInt[30] Mill -> GUI actual position Reserve
ComGUIArrayInt[31] Mill -> GUI actual position Reserve

##### ComGUIArrayIntStringActualStep Mill -> GUI
Value 0 = Stringlen
Value 1..99 = Stringdata

##### ComGUIArrayIntStringActualFile GUI -> Mill
Value 0 = Stringlen
Value 1..255 = Stringdata
"""

""" ############################################################################################ """
""" Import python modules """
import time
import datetime
import tkinter as tk
from tkinter import filedialog as fd
from tkinter import messagebox
#from tkinter import ttk
import multiprocessing
#import ctypes

""" ############################################################################################ """
""" Import own modules """
# Own modules import with "as"
# so that later name of module can be changed without changing code.
import PythonProgrammPortalfraese as PPP

""" ############################################################################################ """
""" Global variables """
# Font
LARGE_FONT = ("Verdana", 12)

""" ############################################################################################ """
""" Constants """
SPEEDSPINDLEMIN = 5000 
SPEEDSPINDLEMAX = 25000

""" ############################################################################################ """
""" Create Multitask "MultitaskComTeensy" and shared memory """
# 'i' = INT, 'f' = REAL
ComGUIArrayInt = multiprocessing.Array('i', range(32)) # Shared memory of INT
ComGUIArrayIntStringActualStep = multiprocessing.Array('i', range(100)) # Shared memory of INT
ComGUIArrayIntStringActualFile = multiprocessing.Array('i', range(256)) # Shared memory of INT
lock = multiprocessing.Lock()
MultitaskComTeensy = multiprocessing.Process(target=PPP.main, args=(ComGUIArrayInt, ComGUIArrayIntStringActualStep, ComGUIArrayIntStringActualFile, lock))

""" ############################################################################################ """
""" Function calculate actual position """
# Input 4 bytes, output float with 4 decimal places
def CalculateActualPosition(IntHighHigh, INTHigh, INTLow, INTLowLow):
    # Local variables
    ActualPositionHex = 0x0
    ActualPositionFloat = 0.0
    
    # From 4 Bytes I2C to one HEX-value
    ActualPositionHex = IntHighHigh << 24
    ActualPositionHex = ActualPositionHex | (INTHigh << 16)
    ActualPositionHex = ActualPositionHex | (INTLow << 8)
    ActualPositionHex = ActualPositionHex | INTLowLow
    
    # When actual position is negative, then Two's complement
    #TwoComplement = (-1)*(256 - Value) # 256 = 8 bit value
    #OneComplement = (-1)*(255 - Value)
    if IntHighHigh >> 7 == 1:
        ActualPositionHex = (-1)*(4294967296 - ActualPositionHex) # 4294967296 = 32 bit value, two's complement

    ActualPositionFloat = float(ActualPositionHex) / 10000.0

    return ActualPositionFloat

""" ############################################################################################ """
""" ############################################################################################ """
""" Class tkinter GUI """
""" tkinter GUI Laeuft in einem eigenen thread """
class AppGUI(object):
    def __init__(self, parent):
        """ #################################################################################### """
        """ Constructor """
        """ #################################################################################### """
        """ Mainwindow """
        self.root = parent # Hauptfenster
        self.root.title("Milling machine") # Titel Hauptfenster
        self.root.geometry("800x600") # Hauptfenster Groesse
        self.root.extra = "MainWindow"
        self.root.bind("<Configure>", self.OnResizeMainwindow) # <Configure> = the widget changed
                                                               # size. Methode "OnResizeMainwindow"
                                                               # bei Fenstergroessenaenderung

        self.root.wm_protocol("WM_DELETE_WINDOW", self.Quit) # Aufruf Methode Quit bei Programmende

        self.WidthMainwindow = 500 # Hoehe Hauptfenster
        self.HeightMainwindow = 300 # Breite Hauptfenster

        # Clear shared memory
        with lock:
            for i in range(31):
                ComGUIArrayInt[i] = 0
            for i in range(99):    
                ComGUIArrayIntStringActualStep[i] = 0
            for i in range(255):    
                ComGUIArrayIntStringActualFile[i] = 0

        """ #################################################################################### """
        """ Common variable """
        """ #################################################################################### """             
        self.StringlenLine = [] # List stringlength for each line in G-Code file
        self.CompareMemoryGUIReadActualLineNumberFile = 999999
        
        self.ProgramActive = False
        self.ReadFileActive = False
        self.FileLoadedFirstTime = False
        self.StartTimeProgram = time.time()
        self.ElapsedTimeProgram = 0
        self.ManualMode = False
        self.ManualCommandXPlus = False
        self.ManualCommandXMinus = False
        self.ManualCommandYPlus = False
        self.ManualCommandYMinus = False
        self.ManualCommandZPlus = False
        self.ManualCommandZMinus = False
        self.ManualCommands = 0
        self.SpindleSpeedmanualMode = SPEEDSPINDLEMIN
        self.ClearCounterI2C = False
        self.PulseProgramActiveOff = False
        self.PulseReadFileActiveOff = False

        """ #################################################################################### """
        """ Menu """
        """ #################################################################################### """
        menubar = tk.Menu(self.root)
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Exit", command=self.Quit) # Aufruf Methode Quit bei Programmende
        menubar.add_cascade(label="File", menu=filemenu)
        self.root.config(menu=menubar)

        """ #################################################################################### """
        """ Frame menu, with buttons on Frame menu """
        """ #################################################################################### """
        self.FrameMenu = tk.Frame(master=self.root,
                                  width=260,
                                  height=240,
                                  borderwidth=0,
                                  bg="white")

        self.FrameMenu.extra = "FrameMenu"

        """ Buttons on Frame menu """
        # Button Main, command lambda, to pass more arguments
        self.ButtonFrameMenuMain = tk.Button(master=self.FrameMenu,
                                             text="Main",
                                             activeforeground="green",
                                             command=lambda: self.showFrame(self.FrameMain,
                                                                            "FrameMain"))
        self.ButtonFrameMenuMain.extra = "ButtonFrameMenuMain"

        # Button Manual, command lambda, to pass more arguments
        self.ButtonFrameMenuManual = tk.Button(master=self.FrameMenu,
                                                text="Manual",
                                                activeforeground="green",
                                                command=lambda: self.showFrame(self.FrameManual,
                                                                               "FrameManual"))
        self.ButtonFrameMenuManual.extra = "ButtonFrameMenuManual"

        # Button Settings, command lambda, to pass more arguments
        self.ButtonFrameMenuSettings = tk.Button(master=self.FrameMenu,
                                                text="Settings",
                                                activeforeground="green",
                                                command=lambda: self.showFrame(self.FrameSettings,
                                                                               "FrameSettings"))
        self.ButtonFrameMenuSettings.extra = "ButtonFrameMenuSettings"

        # Button Diagnostic, command lambda, to pass more arguments
        self.ButtonFrameMenuDiagnostic = tk.Button(master=self.FrameMenu,
                                                text="Diagnostic",
                                                activeforeground="green",
                                                command=lambda: self.showFrame(self.FrameDiagnostic,
                                                                               "FrameDiagnostic"))
        self.ButtonFrameMenuDiagnostic.extra = "ButtonFrameMenuDiagnostic"

        # Save the buttons system colors 
        self.ButtonSystemColorBackground = self.ButtonFrameMenuSettings.cget("background")
        self.ButtonSystemColorActiveBackground = self.ButtonFrameMenuSettings.cget("activebackground")

        """ #################################################################################### """
        """ Frame Main, Frame Manual, Frame Settings, Frame Diagnostic """
        """ #################################################################################### """
        """ Class Frameattributes """
        class Frameattributes:
            x = 0
            y = 0
            width = 0
            height = 0
            visible = False

        """ Frame Main """
        self.FrameMain = tk.Frame(master=self.root,
                                         width=260,
                                         height=240,
                                         borderwidth=0,
                                         background="grey")
        self.FrameMain.place_forget() # hide the frame
        self.FrameMain.extra = "FrameMain"
        self.FrameMainAttributes = Frameattributes() # Class Frameattributes

        """ Frame Manual """
        self.FrameManual = tk.Frame(master=self.root,
                                     width=260,
                                     height=240,
                                     borderwidth=0,
                                     background="grey")
        self.FrameManual.place_forget() # hide the frame
        self.FrameManual.extra = "FrameManual"
        self.FrameManualAttributes = Frameattributes() # Class Frameattributes
 
        """ Frame Settings """
        self.FrameSettings = tk.Frame(master=self.root,
                                          width=260,
                                          height=240,
                                          borderwidth=0,
                                          background="grey")
        self.FrameSettings.place_forget() # hide the frame
        self.FrameSettings.extra = "FrameSettings"
        self.FrameSettingsAttributes = Frameattributes() # Class Frameattributes

        """ Frame Diagnostic """
        self.FrameDiagnostic = tk.Frame(master=self.root,
                                          width=260,
                                          height=240,
                                          borderwidth=0,
                                          background="grey")
        self.FrameDiagnostic.place_forget() # hide the frame
        self.FrameDiagnostic.extra = "FrameDiagnostic"
        self.FrameDiagnosticAttributes = Frameattributes() # Class Frameattributes

        """ #################################################################################### """
        """ Objects on Frame Main """
        """ #################################################################################### """
        """ Label actual positions """
        # X
        self.LabelActualPosXTextvariable = tk.StringVar()
        self.LabelActualPosX = tk.Label(master=self.FrameMain,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualPosXTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelActualPosX.extra = "LabelActualPosX"
        
        # Y
        self.LabelActualPosYTextvariable = tk.StringVar()
        self.LabelActualPosY = tk.Label(master=self.FrameMain,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualPosYTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="green2")
        self.LabelActualPosY.extra = "LabelActualPosY"

        # Z
        self.LabelActualPosZTextvariable = tk.StringVar()
        self.LabelActualPosZ = tk.Label(master=self.FrameMain,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualPosZTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="lightblue1")
        self.LabelActualPosZ.extra = "LabelActualPosZ"

        """ #################################################################################### """
        """ Label date time """
        self.LabelDateTimeTextvariable = tk.StringVar()
        self.LabelDateTime = tk.Label(master=self.FrameMain,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelDateTimeTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="white")
        self.LabelDateTime.extra = "LabelDateTime"

        """ #################################################################################### """
        """ Label elapsed time program """
        self.LabelElapsedTimeTextvariable = tk.StringVar()
        self.LabelElapsedTime = tk.Label(master=self.FrameMain,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelElapsedTimeTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="white")
        self.LabelElapsedTime.extra = "LabelElapsedTime"

        """ #################################################################################### """
        """ Label actual file """
        self.LabelActualFileTextvariable = tk.StringVar()
        self.LabelActualFile = tk.Label(master=self.FrameMain,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualFileTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="white")
        self.LabelActualFile.extra = "LabelActualFile"
        self.LabelActualFileTextvariable.set("File: No file select")

        """ #################################################################################### """
        """ Label actual step """
        self.LabelActualStepTextvariable = tk.StringVar()
        self.LabelActualStep = tk.Label(master=self.FrameMain,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualStepTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="white")
        self.LabelActualStep.extra = "LabelActualStep"

        """ #################################################################################### """
        """ Textbox actual file G-Code and scrollbar """
        self.TextboxGCode = tk.Text(master=self.FrameMain,)
        self.ScrollbarGCode = tk.Scrollbar(master=self.FrameMain, orient=tk.VERTICAL, command=self.TextboxGCode.yview)

        self.TextboxGCode.configure(yscrollcommand=self.ScrollbarGCode.set, font=LARGE_FONT)
        self.TextboxGCode.tag_configure("TagTextWidgetGCode", background="yellow") # Tag actual step in the text widget, background = yellow
        
        self.TextboxGCode.extra = "TextboxGCode"
        self.ScrollbarGCode.extra = "ScrollbarGCode"

        """ #################################################################################### """
        """ Button open file """
        self.ButtonOpenFile = tk.Button(master=self.FrameMain,
                                         text="Open file",
                                         activeforeground="green",
                                         command=self.OpenFile)

        self.ButtonOpenFile.extra = "ButtonOpenFile"

        """ #################################################################################### """
        """ Button start """
        self.ButtonStart = tk.Button(master=self.FrameMain,
                                         text="Start",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=self.ButtonStartPressed)

        self.ButtonStart.extra = "ButtonStart"

        """ #################################################################################### """
        """ Objects on Frame Manual """
        """ #################################################################################### """
        """ Label actual positions """
        # X
        self.LabelActualPosXManual = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualPosXTextvariable, # Same textvariable as in main
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelActualPosXManual.extra = "LabelActualPosXManual"
        
        # Y
        self.LabelActualPosYManual = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualPosYTextvariable, # Same textvariable as in main
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="green2")
        self.LabelActualPosYManual.extra = "LabelActualPosYManual"

        # Z
        self.LabelActualPosZManual = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelActualPosZTextvariable, # Same textvariable as in main
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="lightblue1")
        self.LabelActualPosZManual.extra = "LabelActualPosZManual"

        """ #################################################################################### """
        """ Button manual """
        self.ButtonManual = tk.Button(master=self.FrameManual,
                                         text="Manual mode",
                                         activeforeground="green",
                                         command=lambda: self.ButtonManualPressed("ButtonManual"))
        self.ButtonManual.extra = "ButtonManual"

        """ #################################################################################### """
        """ Button movement X/Y/Z off """
        self.ButtonMovementOff = tk.Button(master=self.FrameManual,
                                         text="Off",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonMovementOff"))

        self.ButtonMovementOff.extra = "ButtonMovementOff"

        """ #################################################################################### """
        """ Button X+ """
        self.ButtonXPlus = tk.Button(master=self.FrameManual,
                                         text="X +",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonXPlus"))
        self.ButtonXPlus.extra = "ButtonXPlus"

        """ Button X- """
        self.ButtonXMinus = tk.Button(master=self.FrameManual,
                                         text="X -",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonXMinus"))
        self.ButtonXMinus.extra = "ButtonXMinus"

        """ #################################################################################### """
        """ Button Y+ """
        self.ButtonYPlus = tk.Button(master=self.FrameManual,
                                         text="Y +",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonYPlus"))
        self.ButtonYPlus.extra = "ButtonYPlus"

        """ Button Y- """
        self.ButtonYMinus = tk.Button(master=self.FrameManual,
                                         text="Y -",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonYMinus"))
        self.ButtonYMinus.extra = "ButtonYMinus"

        """ #################################################################################### """
        """ Button Z+ """
        self.ButtonZPlus = tk.Button(master=self.FrameManual,
                                         text="Z +",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonZPlus"))
        self.ButtonZPlus.extra = "ButtonZPlus"

        """ Button Z- """
        self.ButtonZMinus = tk.Button(master=self.FrameManual,
                                         text="Z -",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonZMinus"))
        self.ButtonZMinus.extra = "ButtonZMinus"

        """ #################################################################################### """
        """ Outputs """
        self.LabelOutputs = tk.Label(master=self.FrameManual,
                                         anchor=tk.CENTER, # Align the text center
                                         padx = 10, # Space left
                                         text="Digital outputs",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey"
        self.LabelOutputs.extra = "LabelOutputs"

        """ #################################################################################### """
        """ Spindle """
        self.LabelSpindle = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Spindle M3,M5",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelSpindle.extra = "LabelSpindle"

        """ Button spindle on """
        self.ButtonSpindleOn = tk.Button(master=self.FrameManual,
                                         text="On",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonSpindleOn"))
        self.ButtonSpindleOn.extra = "ButtonSpindleOn"

        """ Button Spindle off """
        self.ButtonSpindleOff = tk.Button(master=self.FrameManual,
                                         text="Off",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonSpindleOff"))
        self.ButtonSpindleOff.extra = "ButtonSpindleOff"

        """ Slider spindle speed """
        self.SliderSpindleSpeed = tk.Scale(master=self.FrameManual,
                                        from_=SPEEDSPINDLEMIN, to=SPEEDSPINDLEMAX,
                                        orient=tk.HORIZONTAL,
                                        state=tk.DISABLED,
                                        command=self.SliderSpindleSpeedChanged)
        self.SliderSpindleSpeed.extra = "SliderSpindleSpeed"

        """ #################################################################################### """
        """ Coolant """
        self.LabelCoolant = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Coolant M8,M9",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelCoolant.extra = "LabelCoolant"

        """ Button coolant on """
        self.ButtonCoolantOn = tk.Button(master=self.FrameManual,
                                         text="On",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonCoolantOn"))
        self.ButtonCoolantOn.extra = "ButtonCoolantOn"

        """ Button coolant off """
        self.ButtonCoolantOff = tk.Button(master=self.FrameManual,
                                         text="Off",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonCoolantOff"))
        self.ButtonCoolantOff.extra = "ButtonCoolantOff"

        """ #################################################################################### """
        """ Clamp """
        self.LabelClamp = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Clamp M10,M11",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelClamp.extra = "LabelClamp"

        """ Button clamp on """
        self.ButtonClampOn = tk.Button(master=self.FrameManual,
                                         text="On",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonClampOn"))
        self.ButtonClampOn.extra = "ButtonClampOn"

        """ Button clamp off """
        self.ButtonClampOff = tk.Button(master=self.FrameManual,
                                         text="Off",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonClampOff"))
        self.ButtonClampOff.extra = "ButtonClampOff"

        """ #################################################################################### """
        """ Light """
        self.LabelLight = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Light M35,M36",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelLight.extra = "LabelLight"

        """ Button light on """
        self.ButtonLightOn = tk.Button(master=self.FrameManual,
                                         text="On",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonLightOn"))
        self.ButtonLightOn.extra = "ButtonLightOn"

        """ Button light off """
        self.ButtonLightOff = tk.Button(master=self.FrameManual,
                                         text="Off",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonLightOff"))
        self.ButtonLightOff.extra = "ButtonLightOff"

        """ #################################################################################### """
        """ Label inputs: reference sensors, emergency stop, tool length sensor """
        self.LabelInputs = tk.Label(master=self.FrameManual,
                                         anchor=tk.CENTER, # Align the text center
                                         padx = 10, # Space left
                                         text="Digital inputs",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey"
        self.LabelInputs.extra = "LabelInputs"

        self.LabelReferenceSensorX = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Reference sensor X",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelReferenceSensorX.extra = "LabelReferenceSensorX"

        self.LabelReferenceSensorY = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Reference sensor Y",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelReferenceSensorY.extra = "LabelReferenceSensorY"

        self.LabelReferenceSensorZ = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Reference sensor Z",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelReferenceSensorZ.extra = "LabelReferenceSensorZ"

        self.LabelEmergencyStop = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Emergency stop",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelEmergencyStop.extra = "LabelEmergencyStop"

        self.LabelToolLengthSensor = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Tool length sensor",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelToolLengthSensor.extra = "LabelToolLengthSensor"

        """ #################################################################################### """
        """ Measure tool length """
        self.MeasureToolLength = tk.Label(master=self.FrameManual,
                                         anchor=tk.CENTER, # Align the text center
                                         padx = 10, # Space left
                                         text="Measure tool length",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey"
        self.MeasureToolLength.extra = "MeasureToolLength"

        self.LabelToolLength = tk.Label(master=self.FrameManual,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         text="Tool length",
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background="red")
        self.LabelToolLength.extra = "LabelToolLength"

        self.ButtonToolLengthOn = tk.Button(master=self.FrameManual,
                                         text="On",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonToolLengthOn"))
        self.ButtonToolLengthOn.extra = "ButtonToolLengthOn"

        self.ButtonToolLengthOff = tk.Button(master=self.FrameManual,
                                         text="Off",
                                         activeforeground="green",
                                         state=tk.DISABLED,
                                         command=lambda: self.ButtonManualPressed("ButtonToolLengthOff"))
        self.ButtonToolLengthOff.extra = "ButtonToolLengthOff"

        """ #################################################################################### """
        """ Objects on Frame Settings """
        """ #################################################################################### """





        """ #################################################################################### """
        """ Objects on Frame Diagnostic """
        """ #################################################################################### """
        """ Labels diagnistic I2C """
        self.LabelI2CReciveGoodTextvariable = tk.StringVar()
        self.LabelI2CReciveGood = tk.Label(master=self.FrameDiagnostic,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelI2CReciveGoodTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey")
        self.LabelI2CReciveGood.extra = "LabelI2CReciveGood"

        self.LabelI2CReciveChecksumErrorTextvariable = tk.StringVar()
        self.LabelI2CReciveChecksumError = tk.Label(master=self.FrameDiagnostic,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelI2CReciveChecksumErrorTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey")
        self.LabelI2CReciveChecksumError.extra = "LabelI2CReciveChecksumError"
        
        self.LabelI2CReciveHardwareErrorTextvariable = tk.StringVar()
        self.LabelI2CReciveHardwareError = tk.Label(master=self.FrameDiagnostic,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelI2CReciveHardwareErrorTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey")
        self.LabelI2CReciveHardwareError.extra = "LabelI2CReciveHardwareError"
        
        self.LabelI2CSendGoodTextvariable = tk.StringVar()
        self.LabelI2CSendGood = tk.Label(master=self.FrameDiagnostic,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelI2CSendGoodTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey")
        self.LabelI2CSendGood.extra = "LabelI2CSendGood"

        self.LabelI2CSendHardwareErrorTextvariable = tk.StringVar()
        self.LabelI2CSendHardwareError = tk.Label(master=self.FrameDiagnostic,
                                         anchor=tk.W, # Align the text left (West)
                                         padx = 10, # Space left
                                         textvariable=self.LabelI2CSendHardwareErrorTextvariable,
                                         font=LARGE_FONT,
                                         borderwidth = 4,
                                         background=self.ButtonSystemColorActiveBackground)#"grey")
        self.LabelI2CSendHardwareError.extra = "LabelI2CSendHardwareError"

        """ Button Clear counter I2C """
        self.ButtonClearCounterI2C = tk.Button(master=self.FrameDiagnostic,
                                         text="Clear Counter",
                                         activeforeground="green",
                                         command=lambda: self.ButtonDiagnosticPressed("ButtonClearCounterI2C"))
        self.ButtonClearCounterI2C.extra = "ButtonClearCounterI2C"

        """ #################################################################################### """
        """ Call methode poll100. Methode calles itself every 100ms """
        self.poll100()

    """ ######################################################################################## """
    """ End INIT, End Constructor """
    """ ######################################################################################## """

    """ ######################################################################################## """
    """ Methode poll100 """
    def poll100(self):
        # Local variable
        GUIReadActualPos = [0, 0, 0, 0, 0, 0, 0, 0, # 16 Byte actual position axis
                            0, 0, 0, 0, 0, 0, 0, 0]
        ActualPositionFloat = 0.0
        StringActualPos = ""
        StringTmp = ""
        EmergencyStopActive = False
        
        GUIReadActualStep = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, # 100 Byte string actual step
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0,]
        StringActualStep = ""
        TmpValue = 0
        
        GUIReadActualLineNumberFile = 0
        GUIReadDigitalStates = 0
        GUIReadActualState = 0
        StringElapsedTimeProgram = ""
        StringDateTime = ""
        
        GUIReadI2CReciveGood = 0
        GUIReadI2CReciveChecksumError = 0
        GUIReadI2CReciveHardwareError = 0
        GUIReadI2CSendGood = 0
        GUIReadI2CSendHardwareError = 0
        StringDiagnostic = ""
        
        """ Read shared memory """
        with lock:
            # Mill -> GUI read 16 values actual position
            GUIReadActualPos[0] = ComGUIArrayInt[16]
            GUIReadActualPos[1] = ComGUIArrayInt[17]
            GUIReadActualPos[2] = ComGUIArrayInt[18]
            GUIReadActualPos[3] = ComGUIArrayInt[19]
            GUIReadActualPos[4] = ComGUIArrayInt[20]
            GUIReadActualPos[5] = ComGUIArrayInt[21]
            GUIReadActualPos[6] = ComGUIArrayInt[22]
            GUIReadActualPos[7] = ComGUIArrayInt[23]
            GUIReadActualPos[8] = ComGUIArrayInt[24]
            GUIReadActualPos[9] = ComGUIArrayInt[25]
            GUIReadActualPos[10] = ComGUIArrayInt[26]
            GUIReadActualPos[11] = ComGUIArrayInt[27]
            GUIReadActualPos[12] = ComGUIArrayInt[28]
            GUIReadActualPos[13] = ComGUIArrayInt[29]
            GUIReadActualPos[14] = ComGUIArrayInt[30]
            GUIReadActualPos[15] = ComGUIArrayInt[31]
            
            # Mill -> GUI read actual line number in file
            GUIReadActualLineNumberFile = ComGUIArrayInt[8]
            
            # Mill -> GUI State. 0=Idle, 1=Program active, 2=Read file active
            GUIReadActualState = ComGUIArrayInt[9]
            
            # Mill -> GUI Digital states
            GUIReadDigitalStates = ComGUIArrayInt[10]

            # Mill -> GUI Diagnostic data
            GUIReadI2CReciveGood = ComGUIArrayInt[11] # Mill -> GUI Diagnostic I2C Recive good
            GUIReadI2CReciveChecksumError = ComGUIArrayInt[12] # Mill -> GUI Diagnostic I2C Recive checksum error
            GUIReadI2CReciveHardwareError = ComGUIArrayInt[13] # Mill -> GUI Diagnostic I2C Recive hardware error
            GUIReadI2CSendGood = ComGUIArrayInt[14] # Mill -> GUI Diagnostic I2C Send good
            GUIReadI2CSendHardwareError = ComGUIArrayInt[15] # Mill -> GUI Diagnostic I2C Send hardware error           

            # Mill -> GUI read 100 values string actual step
            for i in range(99):
                GUIReadActualStep[i] = ComGUIArrayIntStringActualStep[i]
                
            # GUI -> Mill Mill Spindle speed in manual mode
            ComGUIArrayInt[2] = self.SpindleSpeedmanualMode

        """ Emergency stop button pressed """
        EmergencyStopActive = False
        if GUIReadDigitalStates & 8 != 0: # Bit 3 = Emergency stop button
            EmergencyStopActive = True

        if EmergencyStopActive == True and self.ProgramActive == True:
            with lock:
                ComGUIArrayInt[0] = 3 # 1=Command read file, 2=Command Start program, 3=Command Stop program

        """ Program active, elapsed time duration program """
        # Mill -> GUI State. 0=Idle, 1=Program active, 2=Read file active
        # Program is active
        if GUIReadActualState == 1 and self.ProgramActive == False:
            self.ProgramActive = True
            self.PulseProgramActiveOff = True
            self.StartTimeProgram = time.time()

        # Idle mode
        if GUIReadActualState == 0:
            self.ProgramActive = False
            self.ReadFileActive = False
            
        # Elapsed time
        if self.ProgramActive == True:
            self.ElapsedTimeProgram = int(time.time() - self.StartTimeProgram)

        """ Program active off """
        # Negaive pulse ProgramActive, chage background from orange to system color, set text to start
        if self.PulseProgramActiveOff == True and self.ProgramActive == False:
            self.PulseProgramActiveOff = False
            self.ButtonStart.config(background=self.ButtonSystemColorBackground)
            self.ButtonStart.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonStart.config(text="Start")

        """ Read file active """
        # Mill -> GUI State. 0=Idle, 1=Program active, 2=Read file active
        # Read file is active
        if GUIReadActualState == 2 and self.ReadFileActive == False:
            self.ReadFileActive = True
            self.PulseReadFileActiveOff = True
            # Chage background button open file to orange
            self.ButtonOpenFile.config(background="orange")
            self.ButtonOpenFile.config(activebackground="orange")
            
        """ Read file off """
        # Negaive pulse ReadFileActive, chage background button open file to system color
        if self.PulseReadFileActiveOff == True and self.ReadFileActive == False:
            self.PulseReadFileActiveOff = False
            # Chage background button open file to system color
            self.ButtonOpenFile.config(background=self.ButtonSystemColorBackground)
            self.ButtonOpenFile.config(activebackground=self.ButtonSystemColorActiveBackground)

        """ Disable, enable buttons """
        if self.ProgramActive == True:
            self.ButtonManual['state'] = tk.DISABLED
            self.ButtonOpenFile['state'] = tk.DISABLED
            
        if self.ReadFileActive == True:
            self.ButtonManual['state'] = tk.DISABLED
            self.ButtonStart['state'] = tk.DISABLED            
            
        if self.ManualMode == True:
            self.ButtonOpenFile['state'] = tk.DISABLED
            self.ButtonStart['state'] = tk.DISABLED
            
        if EmergencyStopActive == True:
            self.ButtonStart['state'] = tk.DISABLED

        if self.ReadFileActive == True:
            self.ButtonOpenFile['state'] = tk.DISABLED

        if self.ProgramActive == False and self.ReadFileActive == False:
            self.ButtonManual['state'] = tk.NORMAL
            
        if self.ManualMode == False and self.ProgramActive == False and self.ReadFileActive == False:
            self.ButtonOpenFile['state'] = tk.NORMAL
            
        if self.ManualMode == False and self.ReadFileActive == False and EmergencyStopActive == False and self.FileLoadedFirstTime == True:
            self.ButtonStart['state'] = tk.NORMAL                

        """ Update actual position """
        # In frame main or manual if visible """
        if self.FrameMainAttributes.visible == True or self.FrameManualAttributes.visible == True:

            """ Actual positions """
            # Actual pos X with 4 decimal places
            ActualPositionFloat = CalculateActualPosition(GUIReadActualPos[0], GUIReadActualPos[1], \
                                                          GUIReadActualPos[2], GUIReadActualPos[3])
            StringActualPos = "X: %.4f" % ActualPositionFloat
            self.LabelActualPosXTextvariable.set(StringActualPos)
            
            # Actual pos Y with 4 decimal places
            ActualPositionFloat = CalculateActualPosition(GUIReadActualPos[4], GUIReadActualPos[5], \
                                                          GUIReadActualPos[6], GUIReadActualPos[7])
            StringActualPos = "Y: %.4f" % ActualPositionFloat
            self.LabelActualPosYTextvariable.set(StringActualPos)
            # Actual pos Z with 4 decimal places
            ActualPositionFloat = CalculateActualPosition(GUIReadActualPos[8], GUIReadActualPos[9], \
                                                          GUIReadActualPos[10], GUIReadActualPos[11])
            StringActualPos = "Z: %.4f" % ActualPositionFloat 
            self.LabelActualPosZTextvariable.set(StringActualPos)

        """ Update frame main if visible """
        if self.FrameMainAttributes.visible == True:
            
            """ Actual step """
            # Actual step in Label
            for i in range(1, GUIReadActualStep[0]+1):
                TmpValue = GUIReadActualStep[i]
                StringActualStep = StringActualStep + chr(TmpValue)
            StringTmp = StringActualStep.strip() # Strip, delete whitespaces at the end of a string
            self.LabelActualStepTextvariable.set(StringTmp)

            """ Elapsed time program """
            StringElapsedTimeProgram = ('Elapsed time: {:02d}:{:02d}:{:02d}'.format(self.ElapsedTimeProgram // 3600, (self.ElapsedTimeProgram % 3600 // 60), self.ElapsedTimeProgram % 60))
            self.LabelElapsedTimeTextvariable.set(StringElapsedTimeProgram)
    
            """ Date time """
            d = datetime.datetime.now()
            StringDateTime = d.strftime("%A %Y-%m-%d %H:%M:%S") # Weekday, year, month, day, hour, minute, second
            self.LabelDateTimeTextvariable.set(StringDateTime)

            """ Textbox G-Code """
            # Highlight line actual step in text widget
            if GUIReadActualLineNumberFile != self.CompareMemoryGUIReadActualLineNumberFile:
                self.CompareMemoryGUIReadActualLineNumberFile = GUIReadActualLineNumberFile
    
                if len(self.StringlenLine) > 0 \
                and(GUIReadActualLineNumberFile < len(self.StringlenLine)): # Is len(List StringlenLine) > 0 and < linenumber 
                    self.TextboxGCode.configure(state='normal') # Enable the text widget
    
                    self.TextboxGCode.tag_remove("TagTextWidgetGCode", 1.0, "end") # Remove the actual tag in the text widget
                    #tagstart = "1.0" Example tagstart, Line 1, character 0
                    #tagend = "1.3"   Example tagend, line 1, character 3
                    tagstart = "%d.0" % (GUIReadActualLineNumberFile + 1)
                    tagend = "%d.%d" % ((GUIReadActualLineNumberFile + 1), self.StringlenLine[GUIReadActualLineNumberFile]-1)
                    self.TextboxGCode.tag_add("TagTextWidgetGCode", tagstart, tagend) # Set the tag in the text widget. Whole actual line
        
                    self.TextboxGCode.mark_set("MarkTextWidgetGCode", tagstart) # Set cursor position
                    self.TextboxGCode.see("MarkTextWidgetGCode") # Scroll to the given index
        
                    self.TextboxGCode.configure(state='disabled') # Disable the widget

        """ Update frame manual if visible """
        if self.FrameManualAttributes.visible == True:

            """ Digital states """
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
            if GUIReadDigitalStates & 1 != 0: # Bit 0 = Reference sensor X
                self.LabelReferenceSensorX.config(bg="green")
            else:
                self.LabelReferenceSensorX.config(bg="red")            
            
            if GUIReadDigitalStates & 2 != 0: # Bit 1 = Reference sensor Y
                self.LabelReferenceSensorY.config(bg="green")
            else:
                self.LabelReferenceSensorY.config(bg="red")            
            
            if GUIReadDigitalStates & 4 != 0: # Bit 2 = Reference sensor Z
                self.LabelReferenceSensorZ.config(bg="green")
            else:
                self.LabelReferenceSensorZ.config(bg="red")           

            if GUIReadDigitalStates & 8 != 0: # Bit 3 = Emergency stop button
                self.LabelEmergencyStop.config(bg="green")
            else:
                self.LabelEmergencyStop.config(bg="red")            
            
            if GUIReadDigitalStates & 16 != 0: # Bit 4 = Spindle on
                self.LabelSpindle.config(bg="green")
            else:
                self.LabelSpindle.config(bg="red")
                
            if GUIReadDigitalStates & 32 != 0: # Bit 5 = Coolant on
                self.LabelCoolant.config(bg="green")
            else:
                self.LabelCoolant.config(bg="red")            
            
            if GUIReadDigitalStates & 64 != 0: # Bit 6 = Clamp on
                self.LabelClamp.config(bg="green")
            else:
                self.LabelClamp.config(bg="red")

            if GUIReadDigitalStates & 128 != 0: # Bit 7 = Light on
                self.LabelLight.config(bg="green")
            else:
                self.LabelLight.config(bg="red")

            if GUIReadDigitalStates & 256 != 0: # Bit 8 = Tool length sensor
                self.LabelToolLengthSensor.config(bg="green")
            else:
                self.LabelToolLengthSensor.config(bg="red")        

            if GUIReadDigitalStates & 512 != 0: # Bit 9 = Measure tool length done
                self.ManualCommands = self.ManualCommands & 0b1110_1111_1111_1111 # Reset bit 12, Bit 12 = Measure tool length
                self.ButtonToolLengthOn.config(background=self.ButtonSystemColorBackground)
                self.ButtonToolLengthOn.config(activebackground=self.ButtonSystemColorActiveBackground)
                
                with lock:
                    ComGUIArrayInt[1] = self.ManualCommands

        """ Update frame settings if visible """
        #if self.FrameSettingsAttributes.visible == True:



        """ Update frame diagnostic if visible """
        if self.FrameDiagnosticAttributes.visible == True:
            StringDiagnostic = "I2C Recive good: %d" % GUIReadI2CReciveGood
            self.LabelI2CReciveGoodTextvariable.set(StringDiagnostic)

            StringDiagnostic = "I2C Recive checksum error: %d" % GUIReadI2CReciveChecksumError
            self.LabelI2CReciveChecksumErrorTextvariable.set(StringDiagnostic)

            StringDiagnostic = "I2C Recive hardware error: %d" % GUIReadI2CReciveHardwareError
            self.LabelI2CReciveHardwareErrorTextvariable.set(StringDiagnostic)

            StringDiagnostic = "I2C Send good: %d" % GUIReadI2CSendGood
            self.LabelI2CSendGoodTextvariable.set(StringDiagnostic)

            StringDiagnostic = "I2C Send hardware error: %d" % GUIReadI2CSendHardwareError
            self.LabelI2CSendHardwareErrorTextvariable.set(StringDiagnostic)

        """ Call own methode after 1000ms """
        self.root.after(100, self.poll100)

    """ ######################################################################################## """
    """ Methode OnResizeMainwindow """
    def OnResizeMainwindow(self, event):
        if event.widget.extra == "MainWindow":
            self.WidthMainwindow = event.width
            self.HeightMainwindow = event.height

            """ Place Frame Menue and the buttons on this Frame """
            WidthMenu = self.WidthMainwindow / 100 * 10 # 10% of the MainWindow is for for the menu
            self.FrameMenu.place(x=0, y=0, width=WidthMenu, height=event.height)            
            self.ButtonFrameMenuMain.place(x=5, y=10, width=WidthMenu -10, height=40)
            self.ButtonFrameMenuManual.place(x=5, y=60, width=WidthMenu -10, height=40)
            self.ButtonFrameMenuSettings.place(x=5, y=110, width=WidthMenu -10, height=40)
            self.ButtonFrameMenuDiagnostic.place(x=5, y=160, width=WidthMenu -10, height=40)

            """ Place FrameMain """
            if self.FrameMainAttributes.visible == True:
                self.showFrame(self.FrameMain, "FrameMain")

            """ Place FrameManual """
            if self.FrameManualAttributes.visible == True:
                self.showFrame(self.FrameManual, "FrameManual")

            """ Place FrameSettings """
            if self.FrameSettingsAttributes.visible == True:
                self.showFrame(self.FrameSettings, "FrameSettings")

            """ Place FrameDiagnostic """
            if self.FrameDiagnosticAttributes.visible == True:
                self.showFrame(self.FrameDiagnostic, "FrameDiagnostic")

    """ ######################################################################################## """
    """ Methode show frame "place" """
    def showFrame(self, Frame, obj):
        """ Show the frame """
        FrameX = self.WidthMainwindow / 100 * 10 # 10% of the MainWindow is for for the menu
        FrameY = 0
        FrameWidth = self.WidthMainwindow - self.WidthMainwindow / 100 * 10
        FrameHeight = self.HeightMainwindow
        
        Frame.place(x=FrameX,
                    y=FrameY,
                    width=FrameWidth,
                    height=FrameHeight)
        
        """ Space and place definitions """
        Space = 10
        WidgetWidth = (FrameWidth-(Space*5)) / 4 # Place for 4 widgets horizontal
        WidgetWidthSmall = (WidgetWidth / 2) - (Space / 2)
        WidgetHeight = (FrameHeight-(Space*17)) / 16 # Place for 16 widgets vertical
        ScrollbarWidth = 30

        """ FrameMain is visible """
        if obj == "FrameMain":
            self.FrameMainAttributes.visible = True
            self.FrameManualAttributes.visible = False
            self.FrameSettingsAttributes.visible = False
            self.FrameDiagnosticAttributes.visible = False
            self.hideFrame(self.FrameManual) # Call function "hideFrame"
            self.hideFrame(self.FrameSettings) # Call function "hideFrame"
            self.hideFrame(self.FrameDiagnostic) # Call function "hideFrame"
            # Color the buttons on the Frame menue
            self.ButtonFrameMenuMain.config(background="orange")
            self.ButtonFrameMenuMain.config(activebackground="orange")
            self.ButtonFrameMenuManual.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuManual.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuSettings.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuSettings.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuDiagnostic.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuDiagnostic.config(activebackground=self.ButtonSystemColorActiveBackground)
            # Place Widgets on this frame
            self.LabelActualPosX.place(x=Space, y=Space, width=WidgetWidth, height=WidgetHeight)
            self.LabelActualPosY.place(x=Space, y=(Space*2) + WidgetHeight, width=WidgetWidth, height=WidgetHeight)
            self.LabelActualPosZ.place(x=Space, y=(Space*3) + (WidgetHeight*2), width=WidgetWidth, height=WidgetHeight)
            
            self.LabelDateTime.place(x=(Space*3) + (WidgetWidth*2), y=Space, \
                                     width=(WidgetWidth*2) + Space, height=WidgetHeight)
            self.LabelElapsedTime.place(x=(Space*3) + (WidgetWidth*2), y=(Space*2) + WidgetHeight, \
                                        width=(WidgetWidth*2) + Space, height=WidgetHeight)
            
            self.LabelActualFile.place(x=Space, y=(Space*4) + (WidgetHeight*3), \
                                       width=FrameWidth - (2*Space), height=WidgetHeight)
            self.LabelActualStep.place(x=Space, y=(Space*5) + (WidgetHeight*4), \
                                       width=FrameWidth - (2*Space), height=WidgetHeight)
            
            self.ButtonOpenFile.place(x=Space, y=(Space*6) + (WidgetHeight*5), \
                                      width=WidgetWidth, height=WidgetHeight)
            self.ButtonStart.place(x=(Space *2) + WidgetWidth, y=(Space*6) + (WidgetHeight*5), \
                                   width=WidgetWidth, height=WidgetHeight)
            
            self.TextboxGCode.place(x=Space, y=(Space*7) + (WidgetHeight*6), \
                                    width=FrameWidth - (2*Space) - ScrollbarWidth, height=FrameHeight - ((Space*8) + (WidgetHeight*6)))
            self.ScrollbarGCode.place(x=FrameWidth - Space - ScrollbarWidth + 2, y=(Space*7) + (WidgetHeight*6), \
                                      width=ScrollbarWidth, height=FrameHeight - ((Space*8) + (WidgetHeight*6)))

        """ FrameManual is visible """
        if obj == "FrameManual":
            self.FrameManualAttributes.visible = True
            self.FrameMainAttributes.visible = False
            self.FrameSettingsAttributes.visible = False
            self.FrameDiagnosticAttributes.visible = False
            self.hideFrame(self.FrameMain) # Call function "hideFrame"
            self.hideFrame(self.FrameSettings) # Call function "hideFrame"
            self.hideFrame(self.FrameDiagnostic) # Call function "hideFrame"
            # Color the buttons on the Frame menue
            self.ButtonFrameMenuMain.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuMain.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuManual.config(background="orange")
            self.ButtonFrameMenuManual.config(activebackground="orange")
            self.ButtonFrameMenuSettings.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuSettings.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuDiagnostic.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuDiagnostic.config(activebackground=self.ButtonSystemColorActiveBackground)
            # Place Widgets on this frame
            self.ButtonManual.place(x=Space, y=Space, width=WidgetWidth, height=WidgetHeight)
            self.LabelActualPosXManual.place(x=Space, y=(Space*2) + WidgetHeight, width=WidgetWidth, height=WidgetHeight)
            self.LabelActualPosYManual.place(x=Space, y=(Space*3) + (WidgetHeight*2), width=WidgetWidth, height=WidgetHeight)
            self.LabelActualPosZManual.place(x=Space, y=(Space*4) + (WidgetHeight*3), width=WidgetWidth, height=WidgetHeight)            

            self.ButtonMovementOff.place(x=(Space*4) + WidgetWidth + (WidgetWidthSmall*2), y=(Space*2) + WidgetHeight, \
                                    width=WidgetWidthSmall, height=(WidgetHeight*3) + (Space*2))

            self.ButtonXPlus.place(x=(Space*2) + WidgetWidth, y=(Space*2) + WidgetHeight, \
                                   width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonXMinus.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*2) + WidgetHeight, \
                                    width=WidgetWidthSmall, height=WidgetHeight)
            
            self.ButtonYPlus.place(x=(Space*2) + WidgetWidth, y=(Space*3) + (WidgetHeight*2), \
                                   width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonYMinus.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*3) + (WidgetHeight*2), \
                                    width=WidgetWidthSmall, height=WidgetHeight)
            
            self.ButtonZPlus.place(x=(Space*2) + WidgetWidth, y=(Space*4) + (WidgetHeight*3), \
                                   width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonZMinus.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*4) + (WidgetHeight*3), \
                                    width=WidgetWidthSmall, height=WidgetHeight)    

            self.LabelOutputs.place(x=Space, y=(Space*6) + (WidgetHeight*5), width=(WidgetWidth * 2) + Space, height=WidgetHeight)
            self.LabelSpindle.place(x=Space, y=(Space*7) + (WidgetHeight*6), width=WidgetWidth, height=WidgetHeight)
            self.ButtonSpindleOn.place(x=(Space*2) + WidgetWidth, y=(Space*7) + (WidgetHeight*6), \
                                       width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonSpindleOff.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*7) + (WidgetHeight*6), \
                                        width=WidgetWidthSmall, height=WidgetHeight)    
            self.SliderSpindleSpeed.place(x=Space, y=(Space*8) + (WidgetHeight*7), width=(WidgetWidth * 2) + Space, height=WidgetHeight)

            self.LabelCoolant.place(x=Space, y=(Space*9) + (WidgetHeight*8), width=WidgetWidth, height=WidgetHeight)
            self.ButtonCoolantOn.place(x=(Space*2) + WidgetWidth, y=(Space*9) + (WidgetHeight*8), \
                                       width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonCoolantOff.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*9) + (WidgetHeight*8), \
                                        width=WidgetWidthSmall, height=WidgetHeight)    

            self.LabelClamp.place(x=Space, y=(Space*10) + (WidgetHeight*9), width=WidgetWidth, height=WidgetHeight)
            self.ButtonClampOn.place(x=(Space*2) + WidgetWidth, y=(Space*10) + (WidgetHeight*9), \
                                     width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonClampOff.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*10) + (WidgetHeight*9), \
                                      width=WidgetWidthSmall, height=WidgetHeight)    
            
            self.LabelLight.place(x=Space, y=(Space*11) + (WidgetHeight*10), width=WidgetWidth, height=WidgetHeight)
            self.ButtonLightOn.place(x=(Space*2) + WidgetWidth, y=(Space*11) + (WidgetHeight*10), \
                                     width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonLightOff.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*11) + (WidgetHeight*10), \
                                      width=WidgetWidthSmall, height=WidgetHeight)               
            # Digital inputs
            self.LabelInputs.place(x=(Space*4) + (WidgetWidth*3), y=Space, \
                                   width=WidgetWidth, height=WidgetHeight)
            self.LabelReferenceSensorX.place(x=(Space*4) + (WidgetWidth*3), y=(Space*2) + WidgetHeight, \
                                             width=WidgetWidth, height=WidgetHeight)
            self.LabelReferenceSensorY.place(x=(Space*4) + (WidgetWidth*3), y=(Space*3) + (WidgetHeight*2), \
                                             width=WidgetWidth, height=WidgetHeight)
            self.LabelReferenceSensorZ.place(x=(Space*4) + (WidgetWidth*3), y=(Space*4) + (WidgetHeight*3), \
                                             width=WidgetWidth, height=WidgetHeight)           
            self.LabelEmergencyStop.place(x=(Space*4) + (WidgetWidth*3), y=(Space*5) + (WidgetHeight*4), \
                                          width=WidgetWidth, height=WidgetHeight)                       
            self.LabelToolLengthSensor.place(x=(Space*4) + (WidgetWidth*3), y=(Space*6) + (WidgetHeight*5), \
                                         width=WidgetWidth, height=WidgetHeight)                

            # Measure tool length
            self.MeasureToolLength.place(x=Space, y=(Space*13) + (WidgetHeight*12), width=(WidgetWidth * 2) + Space, height=WidgetHeight)

            self.LabelToolLength.place(x=Space, y=(Space*14) + (WidgetHeight*13), width=WidgetWidth, height=WidgetHeight)
            self.ButtonToolLengthOn.place(x=(Space*2) + WidgetWidth, y=(Space*14) + (WidgetHeight*13), \
                                     width=WidgetWidthSmall, height=WidgetHeight)
            self.ButtonToolLengthOff.place(x=(Space*3) + WidgetWidth + WidgetWidthSmall, y=(Space*14) + (WidgetHeight*13), \
                                      width=WidgetWidthSmall, height=WidgetHeight)   

        """ FrameSettings is visible """
        if obj == "FrameSettings":
            self.FrameSettingsAttributes.visible = True
            self.FrameMainAttributes.visible = False
            self.FrameManualAttributes.visible = False
            self.FrameDiagnosticAttributes.visible = False
            self.hideFrame(self.FrameMain) # Call function "hideFrame"
            self.hideFrame(self.FrameManual) # Call function "hideFrame"
            self.hideFrame(self.FrameDiagnostic) # Call function "hideFrame"
            # Color the buttons on the Frame menue
            self.ButtonFrameMenuMain.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuMain.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuManual.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuManual.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuSettings.config(background="orange")
            self.ButtonFrameMenuSettings.config(activebackground="orange")
            self.ButtonFrameMenuDiagnostic.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuDiagnostic.config(activebackground=self.ButtonSystemColorActiveBackground)
            # Place Widgets on this frame



        """ FrameDiagnostic is visible """
        if obj == "FrameDiagnostic":
            self.FrameDiagnosticAttributes.visible = True
            self.FrameMainAttributes.visible = False
            self.FrameManualAttributes.visible = False
            self.FrameSettingsAttributes.visible = False
            self.hideFrame(self.FrameMain) # Call function "hideFrame"
            self.hideFrame(self.FrameManual) # Call function "hideFrame"
            self.hideFrame(self.FrameSettings) # Call function "hideFrame"
            # Color the buttons on the Frame menue
            self.ButtonFrameMenuMain.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuMain.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuManual.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuManual.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuSettings.config(background=self.ButtonSystemColorBackground)
            self.ButtonFrameMenuSettings.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonFrameMenuDiagnostic.config(background="orange")
            self.ButtonFrameMenuDiagnostic.config(activebackground="orange")
            # Place Widgets on this frame
            self.LabelI2CReciveGood.place(x=Space, y=Space, \
                                          width=WidgetWidth*2, height=WidgetHeight)
            self.LabelI2CReciveChecksumError.place(x=Space, y=(Space*2) + WidgetHeight, \
                                                   width=WidgetWidth*2, height=WidgetHeight)
            self.LabelI2CReciveHardwareError.place(x=Space, y=(Space*3) + (WidgetHeight*2), \
                                                   width=WidgetWidth*2, height=WidgetHeight)
            self.LabelI2CSendGood.place(x=(Space*3) + (WidgetWidth*2), y=Space, \
                                        width=WidgetWidth*2, height=WidgetHeight)
            self.LabelI2CSendHardwareError.place(x=(Space*3) + (WidgetWidth*2), y=(Space*2) + WidgetHeight, \
                                                 width=WidgetWidth*2, height=WidgetHeight)
            self.ButtonClearCounterI2C.place(x=(Space*3) + (WidgetWidth*2), y=(Space*3) + (WidgetHeight*2), \
                                             width=WidgetWidth*2, height=WidgetHeight)                          

    """ ######################################################################################## """
    """ Methode hide frame "place_forget" """
    def hideFrame(self, Frame):
        Frame.place_forget() # hide the frame

    """ ######################################################################################## """
    """ Function Button open file pressed """
    def OpenFile(self):
        Filename = ""
        FilenameStringlen = 0
        Filename = fd.askopenfilename(title = "Select G-Code file")

        # Button cancel pressed
        if not Filename:
            return

        # Transfer stringlen, max 254
        FilenameStringlen = len(Filename)
        if FilenameStringlen <= 254:
            self.LabelActualFileTextvariable.set("File: " + Filename)

            # Write shared memory filename
            with lock:
                ComGUIArrayIntStringActualFile[0] = FilenameStringlen
                # Actual file, ASCII characters to INT values
                for i in range(FilenameStringlen):
                    Letter = Filename[i]
                    ComGUIArrayIntStringActualFile[i+1] = ord(Letter)
                
                time.sleep(0.1)
                ComGUIArrayInt[0] = 1 # 1=Command read file, 2=Command Start program, 3=Command Stop program

            # Fill text widget. Open file readable only
            self.TextboxGCode.configure(state='normal') # Enable the text widget
            self.TextboxGCode.delete("1.0","end") # Delete the whole text in the text widget.
                                         # 1.0 X 1 = row position y 0 = column position

            self.StringlenLine.clear() # Clear list with the stringlength of each line

            with open(Filename, 'r') as f: # Open file readable only
                # Go to each line in the file
                for line in f:
                    self.TextboxGCode.insert(tk.END, line) # Fill the text widget line by line
                    self.StringlenLine.append(len(line)) # Fill list with the stringlength of each line
    
            f.close() # Close file
            self.TextboxGCode.configure(state='disabled') # Disable the widget
            self.CompareMemoryGUIReadActualLineNumberFile = 999999 # Highlight line actual step in text widget
            self.FileLoadedFirstTime = True

        else:
            messagebox.showerror("Error", "File name too long")

    """ ######################################################################################## """
    """ Function Button start pressed """
    def ButtonStartPressed(self):
        
        TmpValue = 0
        
        if self.ProgramActive == False:
            self.ButtonStart.config(background="orange")
            self.ButtonStart.config(activebackground="orange")
            self.ButtonStart.config(text="Abort")
            TmpValue = 2 # Command Start program
        else:
            self.ButtonStart.config(background=self.ButtonSystemColorBackground)
            self.ButtonStart.config(activebackground=self.ButtonSystemColorActiveBackground)
            TmpValue = 3 # Command Stop program
        
        with lock:
            ComGUIArrayInt[0] = TmpValue # 1=Command read file, 2=Command Start program, 3=Command Stop program

    """ ######################################################################################## """
    """ Function Button on frame manual pressed """
    def ButtonManualPressed(self, obj):
        # ManualCommands, ComGUIArrayInt[1]
        # Bit 0 = Manual command Spindle on
        # Bit 1 = Manual command Coolant on
        # Bit 2 = Manual command Clamp on
        # Bit 3 = Manual command Light on
        # Bit 4 = Manual command X+
        # Bit 5 = Manual command X-
        # Bit 6 = Manual command Y+
        # Bit 7 = Manual command Y-
        # Bit 8 = Manual command Z+
        # Bit 9 = Manual command Z-
        # Bit 10 = ClearDiagnosticCounterI2C
        # Bit 11 = Manual mode selected
        # Bit 12 = Measure tool length

        # Local variables
        SwitchManual = 0
        XPlusOffActive = False
        XMinusOffActive = False
        YPlusOffActive = False
        YMinusOffActive = False
        ZPlusOffActive = False
        ZMinusOffActive = False

        # Button Manual Mode
        if obj == "ButtonManual" and self.ManualMode == False :
            SwitchManual = 1
            
        if obj == "ButtonManual" and self.ManualMode == True :
            SwitchManual = 2            
            
        if SwitchManual == 1: # Manual mode on
            self.ManualMode = True
            self.ManualCommandXPlus = False
            self.ManualCommandXMinus = False
            self.ManualCommandYPlus = False
            self.ManualCommandYMinus = False            
            self.ManualCommandZPlus = False
            self.ManualCommandZMinus = False            
            self.ManualCommands = self.ManualCommands | 2048 # Set bit 11 Manual mode selected
            self.ButtonManual.config(background="orange")
            self.ButtonManual.config(activebackground="orange")
            self.ButtonMovementOff['state'] = tk.NORMAL # Enable the manual buttons
            self.ButtonXPlus['state'] = tk.NORMAL
            self.ButtonXMinus['state'] = tk.NORMAL
            self.ButtonYPlus['state'] = tk.NORMAL
            self.ButtonYMinus['state'] = tk.NORMAL            
            self.ButtonZPlus['state'] = tk.NORMAL
            self.ButtonZMinus['state'] = tk.NORMAL            
            self.ButtonSpindleOn['state'] = tk.NORMAL
            self.ButtonSpindleOff['state'] = tk.NORMAL
            self.SliderSpindleSpeed['state'] = tk.NORMAL 
            self.ButtonCoolantOn['state'] = tk.NORMAL
            self.ButtonCoolantOff['state'] = tk.NORMAL
            self.ButtonClampOn['state'] = tk.NORMAL
            self.ButtonClampOff['state'] = tk.NORMAL
            self.ButtonLightOn['state'] = tk.NORMAL
            self.ButtonLightOff['state'] = tk.NORMAL
            self.ButtonToolLengthOn['state'] = tk.NORMAL
            self.ButtonToolLengthOff['state'] = tk.NORMAL

        if SwitchManual == 2: # Manual mode off
            self.ManualMode = False
            self.ManualCommandXPlus = False
            self.ManualCommandXMinus = False
            self.ManualCommandYPlus = False
            self.ManualCommandYMinus = False            
            self.ManualCommandZPlus = False
            self.ManualCommandZMinus = False            
            self.ManualCommands = self.ManualCommands & 0b1110_0100_0000_0000 # Reset bit 0..9, 11 and 12
            self.ButtonManual.config(background=self.ButtonSystemColorBackground)
            self.ButtonManual.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonXPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonXMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXMinus.config(activebackground=self.ButtonSystemColorActiveBackground) 
            self.ButtonYPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonYMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYMinus.config(activebackground=self.ButtonSystemColorActiveBackground)             
            self.ButtonZPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonZMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZMinus.config(activebackground=self.ButtonSystemColorActiveBackground)             
            self.ButtonMovementOff['state'] = tk.DISABLED # Disable the manual buttons
            self.ButtonXPlus['state'] = tk.DISABLED
            self.ButtonXMinus['state'] = tk.DISABLED
            self.ButtonYPlus['state'] = tk.DISABLED
            self.ButtonYMinus['state'] = tk.DISABLED        
            self.ButtonZPlus['state'] = tk.DISABLED
            self.ButtonZMinus['state'] = tk.DISABLED
            self.ButtonSpindleOn['state'] = tk.DISABLED
            self.ButtonSpindleOff['state'] = tk.DISABLED
            self.SliderSpindleSpeed['state'] = tk.DISABLED
            self.ButtonCoolantOn['state'] = tk.DISABLED
            self.ButtonCoolantOff['state'] = tk.DISABLED
            self.ButtonClampOn['state'] = tk.DISABLED
            self.ButtonClampOff['state'] = tk.DISABLED
            self.ButtonLightOn['state'] = tk.DISABLED
            self.ButtonLightOff['state'] = tk.DISABLED
            self.ButtonToolLengthOn['state'] = tk.DISABLED
            self.ButtonToolLengthOff['state'] = tk.DISABLED

        """ Digital outputs """
        if obj == "ButtonSpindleOn" and self.ManualMode == True:
            self.ManualCommands = self.ManualCommands | 1 # Set bit 0
            self.ButtonSpindleOn.config(background="orange")
            self.ButtonSpindleOn.config(activebackground="orange")
        if obj == "ButtonSpindleOff" or self.ManualMode == False:
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1111_1110 # Reset bit 0
            self.ButtonSpindleOn.config(background=self.ButtonSystemColorBackground)
            self.ButtonSpindleOn.config(activebackground=self.ButtonSystemColorActiveBackground)
            
        if obj == "ButtonCoolantOn" and self.ManualMode == True:
            self.ManualCommands = self.ManualCommands | 2 # Set bit 1
            self.ButtonCoolantOn.config(background="orange")
            self.ButtonCoolantOn.config(activebackground="orange")
        if obj == "ButtonCoolantOff" or self.ManualMode == False:
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1111_1101 # Reset bit 1           
            self.ButtonCoolantOn.config(background=self.ButtonSystemColorBackground)
            self.ButtonCoolantOn.config(activebackground=self.ButtonSystemColorActiveBackground)
            
        if obj == "ButtonClampOn" and self.ManualMode == True:
            self.ManualCommands = self.ManualCommands | 4 # Set bit 2
            self.ButtonClampOn.config(background="orange")
            self.ButtonClampOn.config(activebackground="orange")
        if obj == "ButtonClampOff" or self.ManualMode == False:
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1111_1011 # Reset bit 2
            self.ButtonClampOn.config(background=self.ButtonSystemColorBackground)
            self.ButtonClampOn.config(activebackground=self.ButtonSystemColorActiveBackground)
            
        if obj == "ButtonLightOn" and self.ManualMode == True:
            self.ManualCommands = self.ManualCommands | 8 # Set bit 3
            self.ButtonLightOn.config(background="orange")
            self.ButtonLightOn.config(activebackground="orange")
        if obj == "ButtonLightOff" or self.ManualMode == False:
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1111_0111 # Reset bit 3
            self.ButtonLightOn.config(background=self.ButtonSystemColorBackground)
            self.ButtonLightOn.config(activebackground=self.ButtonSystemColorActiveBackground)

        """ Axis +/- """
        # X plus off
        if obj == "ButtonXPlus" and self.ManualCommandXPlus == True and self.ManualMode == True:
            self.ManualCommandXPlus = False
            XPlusOffActive = True
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1110_1111 # Reset bit 4
            self.ButtonXPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # X plus on
        if obj == "ButtonXPlus" and self.ManualMode == True and XPlusOffActive == False:
            self.ManualCommandXPlus = True
            self.ManualCommandXMinus = False
            self.ManualCommands = self.ManualCommands | 16 # Set bit 4
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1101_1111 # Reset bit 5
            self.ButtonXPlus.config(background="orange")
            self.ButtonXPlus.config(activebackground="orange")
            self.ButtonXMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXMinus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # X minus off
        if obj == "ButtonXMinus" and self.ManualCommandXMinus == True and self.ManualMode == True:
            self.ManualCommandXMinus = False
            XMinusOffActive = True
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1101_1111 # Reset bit 5
            self.ButtonXMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXMinus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # X minus on
        if obj == "ButtonXMinus" and self.ManualMode == True and XMinusOffActive == False:
            self.ManualCommandXMinus = True
            self.ManualCommandXPlus = False
            self.ManualCommands = self.ManualCommands | 32 # Set bit 5
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1110_1111 # Reset bit 4
            self.ButtonXMinus.config(background="orange")
            self.ButtonXMinus.config(activebackground="orange")
            self.ButtonXPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXPlus.config(activebackground=self.ButtonSystemColorActiveBackground)

        # Y plus off
        if obj == "ButtonYPlus" and self.ManualCommandYPlus == True and self.ManualMode == True:
            self.ManualCommandYPlus = False
            YPlusOffActive = True
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1011_1111 # Reset bit 6
            self.ButtonYPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # Y plus on
        if obj == "ButtonYPlus" and self.ManualMode == True and YPlusOffActive == False:
            self.ManualCommandYPlus = True
            self.ManualCommandYMinus = False
            self.ManualCommands = self.ManualCommands | 64 # Set bit 6
            self.ManualCommands = self.ManualCommands & 0b1111_1111_0111_1111 # Reset bit 7
            self.ButtonYPlus.config(background="orange")
            self.ButtonYPlus.config(activebackground="orange")
            self.ButtonYMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYMinus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # Y minus off
        if obj == "ButtonYMinus" and self.ManualCommandYMinus == True and self.ManualMode == True:
            self.ManualCommandYMinus = False
            YMinusOffActive = True
            self.ManualCommands = self.ManualCommands & 0b1111_1111_0111_1111 # Reset bit 7
            self.ButtonYMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYMinus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # Y minus on
        if obj == "ButtonYMinus" and self.ManualMode == True and YMinusOffActive == False:
            self.ManualCommandYMinus = True
            self.ManualCommandYPlus = False
            self.ManualCommands = self.ManualCommands | 128 # Set bit 7
            self.ManualCommands = self.ManualCommands & 0b1111_1111_1011_1111 # Reset bit 6
            self.ButtonYMinus.config(background="orange")
            self.ButtonYMinus.config(activebackground="orange")
            self.ButtonYPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYPlus.config(activebackground=self.ButtonSystemColorActiveBackground)

        # Z plus off
        if obj == "ButtonZPlus" and self.ManualCommandZPlus == True and self.ManualMode == True:
            self.ManualCommandZPlus = False
            ZPlusOffActive = True
            self.ManualCommands = self.ManualCommands & 0b1111_1110_1111_1111 # Reset bit 8
            self.ButtonZPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # Z plus on
        if obj == "ButtonZPlus" and self.ManualMode == True and ZPlusOffActive == False:
            self.ManualCommandZPlus = True
            self.ManualCommandZMinus = False
            self.ManualCommands = self.ManualCommands | 256 # Set bit 8
            self.ManualCommands = self.ManualCommands & 0b1111_1101_1111_1111 # Reset bit 9
            self.ButtonZPlus.config(background="orange")
            self.ButtonZPlus.config(activebackground="orange")
            self.ButtonZMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZMinus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # Z minus off
        if obj == "ButtonZMinus" and self.ManualCommandZMinus == True and self.ManualMode == True:
            self.ManualCommandZMinus = False
            ZMinusOffActive = True
            self.ManualCommands = self.ManualCommands & 0b1111_1101_1111_1111 # Reset bit 9
            self.ButtonZMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZMinus.config(activebackground=self.ButtonSystemColorActiveBackground)
        # Z minus on
        if obj == "ButtonZMinus" and self.ManualMode == True and ZMinusOffActive == False:
            self.ManualCommandZMinus = True
            self.ManualCommandZPlus = False
            self.ManualCommands = self.ManualCommands | 512 # Set bit 9
            self.ManualCommands = self.ManualCommands & 0b1111_1110_1111_1111 # Reset bit 8
            self.ButtonZMinus.config(background="orange")
            self.ButtonZMinus.config(activebackground="orange")
            self.ButtonZPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
            
        # All movements off
        if obj == "ButtonMovementOff":
            self.ManualCommands = self.ManualCommands & 0b1111_1100_0000_1111 # Reset bit 4..9
            self.ManualCommandXPlus = False
            self.ManualCommandXMinus = False
            self.ManualCommandYPlus = False
            self.ManualCommandYMinus = False            
            self.ManualCommandZPlus = False
            self.ManualCommandZMinus = False            
            self.ButtonXPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonXMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonXMinus.config(activebackground=self.ButtonSystemColorActiveBackground) 
            self.ButtonYPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonYMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonYMinus.config(activebackground=self.ButtonSystemColorActiveBackground)             
            self.ButtonZPlus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZPlus.config(activebackground=self.ButtonSystemColorActiveBackground)
            self.ButtonZMinus.config(background=self.ButtonSystemColorBackground)
            self.ButtonZMinus.config(activebackground=self.ButtonSystemColorActiveBackground)                     

        """ Measure tool length """
        if obj == "ButtonToolLengthOn" and self.ManualMode == True:
            self.ManualCommands = self.ManualCommands | 4096 # Set bit 12
            self.ButtonToolLengthOn.config(background="orange")
            self.ButtonToolLengthOn.config(activebackground="orange")
        if obj == "ButtonToolLengthOff" or self.ManualMode == False:
            self.ManualCommands = self.ManualCommands & 0b1110_1111_1111_1111 # Reset bit 12
            self.ButtonToolLengthOn.config(background=self.ButtonSystemColorBackground)
            self.ButtonToolLengthOn.config(activebackground=self.ButtonSystemColorActiveBackground)

        with lock:
            ComGUIArrayInt[1] = self.ManualCommands

    """ ######################################################################################## """
    """ Function Button on frame diagnostic pressed """
    def ButtonDiagnosticPressed(self, obj):
        SwitchDiagnostic = 0

        # Button clear counter I2C
        if obj == "ButtonClearCounterI2C"  and self.ClearCounterI2C == False :
            SwitchDiagnostic = 1
            
        if obj == "ButtonClearCounterI2C"  and self.ClearCounterI2C == True :
            SwitchDiagnostic = 2        
            
        if SwitchDiagnostic == 1: # Set bit 10 ClearDiagnosticCounterI2C
            self.ClearCounterI2C = True
            self.ManualCommands = self.ManualCommands | 1024 # Set bit 10
            self.ButtonClearCounterI2C.config(background="orange")
            self.ButtonClearCounterI2C.config(activebackground="orange")

        if SwitchDiagnostic == 2: # Reset bit 10 ClearDiagnosticCounterI2C
            self.ClearCounterI2C = False
            self.ManualCommands = self.ManualCommands & 0b1111_1011_1111_1111 # Reset bit 10
            self.ButtonClearCounterI2C.config(background=self.ButtonSystemColorActiveBackground)
            self.ButtonClearCounterI2C.config(activebackground=self.ButtonSystemColorActiveBackground)

        with lock:
            ComGUIArrayInt[1] = self.ManualCommands           

    """ ######################################################################################## """
    """ Slider spindle speed changed """ 
    def SliderSpindleSpeedChanged(self, obj):
        self.SpindleSpeedmanualMode = int(obj)

    """ ######################################################################################## """
    """ Methode Quit """
    """ Alle threads beenden, Hauptfenster zerstoeren """
    def Quit(self):
        self.root.destroy() # Hauptfenster zerstoeren

""" ############################################################################################ """
""" ############################################################################################ """
""" Funktion main """
def main():
    
    # Start Multitask "MultitaskComTeensy"
    MultitaskComTeensy.start()

    # Mainloop tkinter    
    root = tk.Tk() # Hauptfenster root tkinter GUI
    AppGUI(root) # Instanz eigene Klasse fuer tkinter GUI, Input=tk.Tk (root)
    root.mainloop() # tkinter mainloop. Laeuft in einem eigenen thread

if __name__ == '__main__': # Program is being run directly
    main() # Aufruf Funktion main

    # End Multitask "MultitaskComTeensy"
    # MultitaskComTeensy.join() is not functioning, if in Multitask is a "while True:"
    # Therefore first terminate, then join
    MultitaskComTeensy.terminate()
    MultitaskComTeensy.join()
else:
    print("Program is being imported into another module")
""" ############################################################################################ """
""" ############################################################################################ """

