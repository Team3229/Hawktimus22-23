import os, time
import pyautogui as p
import keyboard

openWindow = (1895, 104)
odometryPos = (545, 0)
vidPos = (1391, 0)
openWinPos = (983, 262)
openWinCorner = (400, 853)

p.FAILSAFE = False

def openWin():

    p.hotkey('ctrl', 'right')
    time.sleep(0.1)
    p.press('F11')

    time.sleep(0.5)

    p.click(openWindow)

    p.press('F11')

    time.sleep(3)

    p.hotkey('win', 'ctrl', 't')

os.popen("C:/ProgramData/Microsoft/Windows/Start Menu/Programs/2023 WPILib Tools/Shuffleboard 2023.lnk")

time.sleep(10)

p.hotkey('ctrl', 'o')
time.sleep(2.5)
p.typewrite('shuffleboard.json')
p.press('enter')

os.popen("C:/Users/njman/AppData/Local/Programs/advantagescope/AdvantageScope.exe")

time.sleep(5)

p.press('F11')
p.click(0, 0)
p.click(0, 315)
time.sleep(2.5)
p.press('a')
p.press('down')
p.press('enter')
p.press('F11')

for i in range(8):
    p.hotkey('ctrl', 'left')

# odometry
time.sleep(0.1)
openWin()

p.moveTo(openWinPos)
p.mouseDown()
p.moveRel(1, 0)
p.moveTo(odometryPos)
p.mouseUp()

# camera stream
p.hotkey('alt', 'tab')
time.sleep(0.1)
openWin()

p.moveTo(openWinCorner)
p.mouseDown()
p.moveRel(1, 0)
p.moveRel(310, -23)
p.mouseUp()

p.moveTo(openWinPos)
p.mouseDown()
p.moveRel(1, 0)
p.moveTo(vidPos)
p.mouseUp()

# blue controller
p.hotkey('alt', 'tab')
time.sleep(0.1)
openWin()

p.moveTo(405, openWinCorner[1])
p.mouseDown()
p.moveRel(1, 0)
p.moveTo(p.position(0))
p.mouseUp()

p.moveTo(1509, 849)
p.mouseDown()
p.moveTo(p.position(367))
p.mouseUp()

p.moveTo(50, 230)
p.mouseDown()
p.moveTo(p.position(y=530))
p.mouseUp()

# white controller
p.hotkey('alt', 'tab')
time.sleep(0.5)
openWin()

p.moveTo(405, openWinCorner[1])
p.mouseDown()
p.moveRel(1, 0)
p.moveTo(p.position(0))
p.mouseUp()

p.moveTo(1509, 849)
p.mouseDown()
p.moveTo(p.position(1920))
p.mouseUp()

p.moveTo(openWinPos[0], 230)
p.mouseDown()
p.moveTo(p.position(y=555-25))
p.mouseUp()

p.moveTo(0, openWinCorner[1])
p.mouseDown()
p.moveRel(1, 0)
p.moveTo(1544, openWinCorner[1])
p.mouseUp()

# swerve
p.hotkey('alt', 'tab')
time.sleep(0.5)
openWin()

p.moveTo(1216, 1072)
p.mouseDown()
p.moveRel(-1, 0)
p.moveTo(0, 560)
p.mouseUp()

p.moveTo(463, 243)
p.mouseDown()
p.moveRel(-1, 0)
p.moveRel(1, 0)
p.moveTo(430, 545)
p.mouseUp()

# refocus
p.hotkey('alt', 'tab')
time.sleep(0.2)
p.hotkey('alt', 'esc')
time.sleep(0.1)

p.click(10, 10)
p.click(1910, 10)

p.keyDown('alt')
for i in range(7):
    p.press('tab')
p.keyUp('alt')