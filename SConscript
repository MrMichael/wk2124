
from building import *

cwd     = GetCurrentDir()
src     = Glob('*.c') + Glob('*.cpp')
path    = [cwd]

group = DefineGroup('wk2124', src, depend = ['PKG_USING_WK2124'], CPPPATH = path)

Return('group')
