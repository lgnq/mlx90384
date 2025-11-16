from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add mlx90382 src files.
if GetDepend('PKG_USING_MLX90384'):
    src += Glob('src/mlx90384.c')

if GetDepend('RT_USING_SENSOR'):
    src += Glob('src/sensor_melexis_mlx90384.c')

if GetDepend('PKG_USING_MLX90384_SAMPLE'):
    src += Glob('samples/mlx90384_sample.c')

# add mlx90384 include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('mlx90384', src, depend = ['PKG_USING_MLX90384'], CPPPATH = path)

Return('group')
