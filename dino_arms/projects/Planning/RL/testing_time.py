import ast
import numpy as np
from timeit import timeit
import math

expression="[1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
exp_as_func = eval('lambda: ' + expression)
# exp_as_func()

# for i in range(100000):
# print(timeit('eval("math.sin(8)*9")', setup="import math", number=100000))
# print(timeit('eval("[1,2,3,4]")', setup="import math", number=100000))
# print(timeit('ast.literal_eval("[1,2,3,4]")', setup="import ast", number=100000))
# print(timeit('eval("lambda: " + "[1,2,3,4]")', setup="import math", number=100000))
print(timeit('exp_as_func()', setup="from __main__ import exp_as_func", number=100000)/100000.0)
# print(timeit('str([1,2,3,3,4,3,5,54,3])', setup="from __main__ import exp_as_func", number=100000)/100000.0)
# print(timeit('np.count_nonzero(exp_as_func())', setup="import numpy as np \nfrom __main__ import exp_as_func", number=100000)/100000.0)
