import numpy as np
from numpy import genfromtxt
exchange = genfromtxt('/home/genki/Desktop/exchange.txt', delimiter=',')
no_exchange = genfromtxt('/home/genki/Desktop/no_exchange.txt', delimiter=',')

print(exchange[:,2])
print(no_exchange[:,2])
a = np.stack([exchange[:,2],no_exchange[:,2]], axis=1)
print(a)
np.savetxt("/home/genki/Desktop/distance_traveled.txt", a, delimiter=",", fmt='%f')