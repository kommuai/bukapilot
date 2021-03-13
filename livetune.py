from selfdrive.livetune_conf import livetune_conf
import os
BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))

letters = { "a":[ "###", "# #", "###", "# #", "# #"], "b":[ "###", "# #", "###", "# #", "###"], "c":[ "###", "#", "#", "#", "###"], "d":[ "##", "# #", "# #", "# #", "##"], "e":[ "###", "#", "###", "#", "###"], "f":[ "###", "#", "###", "#", "#"], "g":[ "###", "# #", "###", "  #", "###"], "h":[ "# #", "# #", "###", "# #", "# #"], "i":[ "###", " #", " #", " #", "###"], "j":[ "###", " #", " #", " #", "##"], "k":[ "# #", "##", "#", "##", "# #"], "l":[ "#", "#", "#", "#", "###"], "m":[ "# #", "###", "###", "# #", "# #"], "n":[ "###", "# #", "# #", "# #", "# #"], "o":[ "###", "# #", "# #", "# #", "###"], "p":[ "###", "# #", "###", "#", "#"], "q":[ "###", "# #", "###", "  #", "  #"], "r":[ "###", "# #", "##", "# #", "# #"], "s":[ "###", "#", "###", "  #", "###"], "t":[ "###", " #", " #", " #", " #"], "u":[ "# #", "# #", "# #", "# #", "###"], "v":[ "# #", "# #", "# #", "# #", " #"], "w":[ "# #", "# #", "# #", "###", "###"], "x":[ "# #", " #", " #", " #", "# #"], "y":[ "# #", "# #", "###", "  #", "###"], "z":[ "###", "  #", " #", "#", "###"], " ":[ " "], "1":[ " #", "##", " #", " #", "###"], "2":[ "###", "  #", "###", "#", "###"], "3":[ "###", "  #", "###", "  #", "###"], "4":[ "#", "#", "# #", "###", "  #"], "5":[ "###", "#", "###", "  #", "###"], "6":[ "###", "#", "###", "# #", "###"], "7":[ "###", "  # ", " #", " #", "#"], "8":[ "###", "# #", "###", "# #", "###"], "9":[ "###", "# #", "###", "  #", "###"], "0":[ "###", "# #", "# #", "# #", "###"], "!":[ " # ", " # ", " # ", "   ", " # "], "?":[ "###", "  #", " ##", "   ", " # "], ".":[ "   ", "   ", "   ", "   ", " # "], "]":[ "   ", "   ", "   ", "  #", " # "], "/":[ "  #", "  #", " # ", "# ", "# "], ":":[ "   ", " # ", "   ", " # ", "   "], "@":[ "###", "# #", "## ", "#  ", "###"], "'":[ " # ", " # ", "   ", "   ", "   "], "#":[ " # ", "###", " # ", "###", " # "], "-":[ "  ", "  ","###","   ","   "] }
# letters stolen from here: http://www.stuffaboutcode.com/2013/08/raspberry-pi-minecraft-twitter.html

def print_letters(text):
    bigletters = []
    for i in text:
        bigletters.append(letters.get(i.lower(),letters[' ']))
    output = ['']*5
    for i in range(5):
        for j in bigletters:
            temp = ' '
            try:
                temp = j[i]
            except:
                pass
            temp += ' '*(5-len(temp))
            temp = temp.replace(' ',' ')
            temp = temp.replace('#','@')
            output[i] += temp
    return '\n'.join(output)
import sys, termios, tty, os, time

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0.2

livetune = livetune_conf()
livetune.conf['tuneGernby'] = "1"
#livetune.write_config(livetune.conf)
param = [ "fakeEngage", "accelerationMode", "Kp", "Ki", "Kf", "steerRatio", "sR_boost", "sR_BP0", \
         "sR_BP1", "sR_time", "slowOnCurves", \
         "1barBP0", "1barBP1", "1barMax", "2barBP0", "2barBP1", \
         "2barMax", "3barBP0", "3barBP1", "3barMax", \
         "1barHwy", "2barHwy", "3barHwy", "maxSteer", "steerDeltaUp", "steerDeltaDown", \
         "steerDriverAllowance", "steerDriverMult"]

j = 0
while True:
  os.system("clear")
  print ("")
  print (param[j])
  print ("")
  print (print_letters(livetune.conf[param[j]]))
  print ("")
  print ("1,3,5,7,r to increase 0.1,0.05,0.01,0.001,0.00001")
  print ("a,d,g,j,v to decrease 0.1,0.05,0.01,0.001,0.00001")
  print ("0 / L / P to make the value 0 / 1 / 2")
  print ("press SPACE / m for next /prev parameter")
  print ("press z to quit")

  char  = getch()
  write_json = False
  if (char == "v"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) - 0.00001),5))
    write_json = True

  if (char == "r"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) + 0.00001),5))
    write_json = True

  if (char == "7"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) + 0.001),5))
    write_json = True

  if (char == "5"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) + 0.01),5))
    write_json = True

  elif (char == "3"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) + 0.05),5))
    write_json = True

  elif (char == "1"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) + 0.1),5))
    write_json = True

  elif (char == "j"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) - 0.001),5))
    write_json = True

  elif (char == "g"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) - 0.01),5))
    write_json = True

  elif (char == "d"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) - 0.05),5))
    write_json = True

  elif (char == "a"):
    livetune.conf[param[j]] = str(round((float(livetune.conf[param[j]]) - 0.1),5))
    write_json = True

  elif (char == "0"):
    livetune.conf[param[j]] = "0"
    write_json = True

  elif (char == "l"):
    livetune.conf[param[j]] = "1"
    write_json = True

  elif (char == "p"):
    livetune.conf[param[j]] = "2"
    write_json = True

  elif (char == " "):
    if j < len(param) - 1:
      j = j + 1
    else:
      j = 0

  elif (char == "m"):
    if j > 0:
      j = j - 1
    else:
      j = len(param) - 1

  elif (char == "z"):
    exit()
    break


  if float(livetune.conf['tuneGernby']) != 1 and float(livetune.conf['tuneGernby']) != 0:
    livetune.conf['tuneGernby'] = "1"

  if float(livetune.conf['accelerationMode']) != 0 and float(livetune.conf['accelerationMode']) != 1 and float(livetune.conf['accelerationMode'] != "2"):
    livetune.conf['accelerationMode'] = "1"

  #if float(livetune.conf['Ki']) < 0 and float(livetune.conf['Ki']) != -1:
    #livetune.conf['Ki'] = "0"

  #if float(livetune.conf['Ki']) > 2:
    #livetune.conf['Ki'] = "2"

  #if float(livetune.conf['Kp']) < 0 and float(livetune.conf['Kp']) != -1:
    #livetune.conf['Kp'] = "0"

  #if float(livetune.conf['Kp']) > 3:
    #livetune.conf['Kp'] = "3"

  #if livetune.conf['liveParams'] != "1" and livetune.conf['liveParams'] != "0":
  #  livetune.conf['liveParams'] = "1"

  if float(livetune.conf['steerRatio']) < 1 and float(livetune.conf['steerRatio']) != -1:
    livetune.conf['steerRatio'] = "1"

  #if float(livetune.conf['steerRateCost']) < 0.01 and float(livetune.conf['steerRateCost']) != -1:
  #  livetune.conf['steerRateCost'] = "0.01"

  #if float(livetune.conf['deadzone']) < 0:
  #  livetune.conf['deadzone'] = "0"

  if float(livetune.conf['1barBP0']) < -0.5:
    livetune.conf['1barBP0'] = "-0.5"

  if float(livetune.conf['1barBP0']) > 0.5:
    livetune.conf['1barBP0'] = "0.5"

  if float(livetune.conf['1barBP1']) < 0.5:
    livetune.conf['1barBP1'] = "0.5"

  if float(livetune.conf['1barBP1']) > 8:
    livetune.conf['1barBP1'] = "8"

  if float(livetune.conf['1barMax']) < 0.9:
    livetune.conf['1barMax'] = "0.9"

  if float(livetune.conf['1barMax']) > 2.5:
    livetune.conf['1barMax'] = "2.5"

  if float(livetune.conf['2barBP0']) < -0.5:
    livetune.conf['2barBP0'] = "-0.5"

  if float(livetune.conf['2barBP0']) > 0.5:
    livetune.conf['2barBP0'] = "0.5"

  if float(livetune.conf['2barBP1']) < 0.5:
    livetune.conf['2barBP1'] = "0.5"

  if float(livetune.conf['2barBP1']) > 8:
    livetune.conf['2barBP1'] = "8"

  if float(livetune.conf['2barMax']) < 1.3:
    livetune.conf['2barMax'] = "1.3"

  if float(livetune.conf['2barMax']) > 2.5:
    livetune.conf['2barMax'] = "2.5"

  if float(livetune.conf['3barBP0']) < -0.5:
    livetune.conf['3barBP0'] = "-0.5"

  if float(livetune.conf['3barBP0']) > 0.5:
    livetune.conf['3barBP0'] = "0.5"

  if float(livetune.conf['3barBP1']) < 0.5:
    livetune.conf['3barBP1'] = "0.5"

  if float(livetune.conf['3barBP1']) > 8:
    livetune.conf['3barBP1'] = "8"

  if float(livetune.conf['3barMax']) < 1.8:
    livetune.conf['3barMax'] = "1.8"

  if float(livetune.conf['3barMax']) > 2.5:
    livetune.conf['3barMax'] = "2.5"

  if float(livetune.conf['1barHwy']) < 0:
    livetune.conf['1barHwy'] = "0"

  if float(livetune.conf['2barHwy']) < 0:
    livetune.conf['2barHwy'] = "0"

  if float(livetune.conf['3barHwy']) < 0:
    livetune.conf['3barHwy'] = "0"

  if float(livetune.conf['1barHwy']) > 2:
    livetune.conf['1barHwy'] = "2"

  if float(livetune.conf['2barHwy']) > 2:
    livetune.conf['2barHwy'] = "2"

  if float(livetune.conf['3barHwy']) > 2:
    livetune.conf['3barHwy'] = "2"

  if float(livetune.conf['Kf']) > 0.01:
    livetune.conf['Kf'] = "0.01"

  if float(livetune.conf['Kf']) < 0:
    livetune.conf['Kf'] = "0"

  if float(livetune.conf['sR_boost']) < 0:
    livetune.conf['sR_boost'] = "0"

  if float(livetune.conf['sR_BP0']) < 0:
    livetune.conf['sR_BP0'] = "0"

  if float(livetune.conf['sR_BP1']) < 0:
    livetune.conf['sR_BP1'] = "0"

  if float(livetune.conf['sR_time']) < 1:
    livetune.conf['sR_time'] = "1"

  #if float(livetune.conf['Kf']) < 0.00001:
  livetune.conf['Kf'] = str("{:.5f}".format(float(livetune.conf['Kf'])))

  if float(livetune.conf['slowOnCurves']) > 0.00001:
    livetune.conf['slowOnCurves'] = "1"

  if float(livetune.conf['slowOnCurves']) <= 0.99999:
    livetune.conf['slowOnCurves'] = "0"

  if write_json:
    livetune.write_config(livetune.conf)

  time.sleep(button_delay)

else:
  exit()
