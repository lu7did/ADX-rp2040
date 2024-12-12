#*--------------------------------------------------------------------
#* nco.py
#* Program to explore and experiment viable divisors for most
#* common HF frequencies based on the selection of a (valid) system
#* clock and the manipulation of the 16 integer divisor bits and 8
#* fractional divisor the rp2040 PIO provides
#*--------------------------------------------------------------------
from math import ceil, floor
import numpy as np

#*--------------------------------------------------------------------
def valid_system_clocks(minimum_frequency, maximum_frequency):
  valid_system_clocks = {}

  #generate a table of all system frequencies in the valid range

  for refdiv in reversed(range(1, 2)):
    reffreq = 12e6/refdiv
    for fbdiv in range(16, 320):
      vco_freq = reffreq*fbdiv
      if vco_freq > 1600e6:
        continue
      if vco_freq < 750e6:
        continue
      for postdiv1 in range(1, 8):
        for postdiv2 in range(1, 8):
          f = vco_freq/(postdiv1*postdiv2)
          if f > maximum_frequency:
            continue
          if f < minimum_frequency:
            continue
          valid_system_clocks[int(f)] = (refdiv, fbdiv, postdiv1, postdiv2)

  print("valid_system_clocks")
  ratios = []
  for i, f in enumerate(sorted(valid_system_clocks.keys())):
    settings = valid_system_clocks[f]
    print("{%u, %u, %u, %u, %u},"%((f,)+settings))
    ratios.append(f/minimum_frequency)

  return valid_system_clocks

#*--------------------------------------------------------------------
def best_clock_frequency(desired_frequency, vsc):
  best_frequency = 1
  for system_clock in vsc:
    best_divider = round(256 * system_clock/desired_frequency) / 256
    actual_frequency = system_clock/best_divider
    if abs(actual_frequency - desired_frequency) < abs(best_frequency - desired_frequency):
      best_frequency = actual_frequency
  return best_frequency

#*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
def getbest(desired_frequency, vsc):

  best_frequency=1
  count=0
  for system_clock in vsc:

     #print("System clock=%f" % (system_clock))
     div=system_clock/desired_frequency
     div1,div2=divmod(div,1)

     div2frac=div2*256
     divf,z=divmod(div2frac,1)
     divf=divf/256
     new_frequency=(system_clock/(div1+divf))
     count+= 1
     if abs(new_frequency - desired_frequency) < abs(best_frequency - desired_frequency):
        old_best=best_frequency
        best_frequency = new_frequency
        delta=best_frequency-desired_frequency
        print("CHANGE desired=%f best=%f new=%f error=%f" % (desired_frequency,old_best,new_frequency,delta))


  print("count(%d) desired frequency=%f obtained frequency=%f difference=%f" % (count,desired_frequency/4,best_frequency/4,(desired_frequency/4-best_frequency/4)))
  return best_frequency
#*--------------------------------------------------------------------         
#* First compute errors if clock is in the range 125-133 MHz
#*--------------------------------------------------------------------
frequency = 28074000
frequencies = []
errors_new = []
vsc = valid_system_clocks(125e6, 270.0e6)
while frequency <= 28074000:
  print("Explore ----> %f" % (frequency))
  best_frequency=getbest(frequency*4,vsc)
  
  #best_frequency = best_clock_frequency(frequency*4, vsc) 

  frequencies.append(frequency)
  errors_new.append(frequency-best_frequency/4)
  #print("Desired Freq=%f best=%f error=%f" % (frequency,best_frequency/4,(frequency-best_frequency/4)))
  frequency += 6.25

#*-------------------------------------------------------------
#* Print and plot results
#*-------------------------------------------------------------
print("Raspberry pico (rp2040) clock simulator")
print("pico2      min=%d max=%d" % (min(errors_new),max(errors_new)))

#*-------------------------------------------------------------
#* Draw results (Graph is best error for a given NCO frequency)
#*-------------------------------------------------------------
from matplotlib import pyplot as plt
plt.title("Best NCO Frequency")
plt.xlabel("Tuned Frequency MHz")
plt.ylabel("Distance from Nearest Hz")
plt.plot(np.array(frequencies)/1e6, np.array(errors_new)/1e3, label="125-133 MHz overclock")
plt.legend()
plt.show()
