import re
import matplotlib.pyplot as plt
import numpy as np

print("Hello World")

# test = np.loadtxt('lab10_data_trial_141.txt')
# print(test.shape())
Matrix = []

# Values to Edit
val_of_interestX = 1 #Usually set to time
val_of_interestY = 3
#Can adjust the script to display both roll and pitch on the same plot
trial = 313

# Code to parse
f = open(f'lab10_data_trial_{trial}.txt', 'r')
lines = f.readlines()
lines_split = lines[0].split(' ')
Matrix.append(lines_split)
print(Matrix[0][val_of_interestY])
X, Y,  = [], []
del lines[0]

#Need to delete the last two lines that are empty either with Python or manually
n = 2
del lines[-2:]

#Place data into lists
for row in lines:
    new_row = row.split(' ')
    X.append(float(new_row[val_of_interestX]))
    Y.append(float(new_row[val_of_interestY]))
# print(t_now)

#Create and Display the plot
plt.plot(X, Y)
plt.xlabel(f"{Matrix[0][val_of_interestX]}")
plt.ylabel(f"{Matrix[0][val_of_interestY]}")
plt.title(f"PID Plot Trial {trial}")
plt.savefig(f"pid_plot_trial_{trial}.png")

plt.show()