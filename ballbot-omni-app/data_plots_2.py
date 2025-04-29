import re
import matplotlib.pyplot as plt
import numpy as np

print("Hello World")

# test = np.loadtxt('lab10_data_trial_141.txt')
# print(test.shape())
Matrix = []

#filter error, filter output of the d - term

# Values to Edit
#14 and 15 for Tpx and Tpy values 
#16 and 17 for Tix and Tiy values 
#18 and 19 for Tdx and Tdy values From 295
#2 and 3 for roll and pitch
val_of_interest_time = 1 #Usually set to time
val_of_interestX = 2
val_of_interestY = 3


#Can adjust the script to display both roll and pitch on the same plot
trial = 610

# Code to parse
# f = open(f'hw5_data_trial_{trial}.txt', 'r')
f = open(f'controller_data_trial_{trial}.txt', 'r')
# f = open(f'controller_data_trial_{trial}.txt')
lines = f.readlines()
lines_split = lines[0].split(' ')
Matrix.append(lines_split)
print(Matrix[0][val_of_interestY])
time, X, Y,  = [], [], []
del lines[0]

#Need to delete the last two lines that are empty either with Python or manually
n = 2
del lines[-2:]

#Place data into lists
for row in lines:
    new_row = row.split(' ')
    time.append(float(new_row[val_of_interest_time]))
    X.append(float(new_row[val_of_interestX]))
    Y.append(-float(new_row[val_of_interestY]))
# print(t_now)

#Create and Display the plot

# plt.plot(time, 0, label = "zero")
plt.plot(time, X, label = "roll")
plt.plot(time, Y, label = "pitch")
plt.xlabel(f"{Matrix[0][val_of_interestX]}")
plt.ylabel(f"{Matrix[0][val_of_interestY]}")
plt.title(f"PID Plot Trial {trial}")
plt.legend()
plt.savefig(f"pid_plot_roll_pitch_trial_{trial}.png")
plt.show()