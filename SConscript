
from building import *

cwd     = GetCurrentDir()
path    = [cwd]
src = Split('''
wk2124s.c
''')
if GetDepend(['RT_USING_SERIAL']):
    if GetDepend(['RT_USING_SERIAL_V1']):
        src += Glob('wk2124_usart.c')
    if GetDepend(['RT_USING_SERIAL_V2']):
    	src += Glob('wk2124_usart_v2.c')
        
group = DefineGroup('wk2124', src, depend = ['RT_USING_SERIAL','PKG_USING_WK2124'], CPPPATH = path)

Return('group')
