import ast
import numpy as np
from timeit import timeit
import math

import matplotlib.pyplot as plt
# expression="[1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
# exp_as_func = eval('lambda: ' + expression)
# print exp_as_func()[:5]

# for i in range(100000):
# print(timeit('eval("math.sin(8)*9")', setup="import math", number=100000))
# print(timeit('eval("[1,2,3,4]")', setup="import math", number=100000))
# print(timeit('ast.literal_eval("[1,2,3,4]")', setup="import ast", number=100000))
# print(timeit('eval("lambda: " + "[1,2,3,4]")', setup="import math", number=100000))
# print(timeit('exp_as_func()', setup="from __main__ import exp_as_func", number=100000)/100000.0)
# print(timeit('str([1,2,3,3,4,3,5,54,3])', setup="from __main__ import exp_as_func", number=100000)/100000.0)
# print(timeit('np.count_nonzero(exp_as_func())', setup="import numpy as np \nfrom __main__ import exp_as_func", number=100000)/100000.0)

e = 1
e_list = []
e2 = 1
e_list_2 = []
lr = .02
lr_list = []
found1 = True
found2 = True
for i in range(1,30000001):
    e *= (1-e*.00000009)
    e2 *= (1-e2*.0000002)
    # e2 *= 9/10
    if i%1000000==0:
        lr *= .95
    e_list.append(e)
    e_list_2.append(e2)
    lr_list.append(lr)
    if e<0.5 and found1:
        found1 = False
        print "e", i
    if e2<0.5 and found2:
        found2 = False
        print "e2", i

    # print i/1000000, e

print e, e2
# print lr
plt.plot(range(30000000), e_list, 'r', range(30000000), e_list_2, 'g')
# plt.plot(range(20000000), lr_list, 'b')
#
plt.axis([0,30000000,0,1])
plt.show()
