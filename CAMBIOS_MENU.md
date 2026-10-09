# Menú de N y bordes con OpenGL 1.2

> Notas históricas: algunos fragmentos describen versiones anteriores. Para las opciones y el estado actuales, consultar README.md y el código fuente.

Se eligió la opción B: menú propio con FreeGLUT, sin vendorizar ImGui ni añadir un framework. Los cambios quedan dentro de las interfaces existentes.

## Archivos

- `include/simulation.hpp`: estructura `SimParams` y método `Dynamics::reconfigure`.
- `src/main.cpp`: panel con selector +/- de N, opciones de borde y reinicio; `needs_reset`; aplicación al principio del frame; picking con GLU. El dibujo existente usa ahora `glBegin/glEnd`, matrices y `glRasterPos`, sin shaders ni VBOs.
- `src/dynamics.cpp`: recreación de los cuatro buffers dependientes de N, conservando contexto/cola/programa/kernels y liberando recursos anteriores. Inicializa todos los buffers y estados host; no se reserva por frame.
- `CMakeLists.txt`: enlace GLU, eliminación de `GL_GLEXT_PROTOTYPES`.
- `README.md`: instrucciones actualizadas.
- `shaders/scene.vert` y `shaders/scene.frag`: eliminados porque ya no se usan.
- `kernels/verlet.cl`, parser, parámetros y captura: permanecen sin cambios funcionales.

## Estado y reset

```cpp
struct SimParams {
    int N = 20;
    bool extremos_fijos = true;
};
static SimParams params;
static bool needs_reset = false;
```

Al iniciar, `params` toma N y borde del archivo. Flechas y botones sólo cambian este estado y activan la bandera. `applyReset()` se llama una vez al comienzo de `display()`:

```cpp
if (!needs_reset) return;
needs_reset = false;
drag = -1;
target = speed = 0;
mouseTime = 0;
Config nextConfig = fileConfig;
nextConfig.N = params.N;
nextConfig.first = nextConfig.last = params.extremos_fijos;
if (!nextConfig.y.empty())
    nextConfig.y.resize(params.N, float(nextConfig.y_eq));
if (!nextConfig.v.empty())
    nextConfig.v.resize(params.N, 0.f);
// Se ajusta modo a las masas móviles; N=2 fijo usa patrón cero.
d->reconfigure(nextConfig);
// Actualiza configuración, perfil, cámara; resetea acumulador y reloj.
```

El código completo incluye manejo del fallo de reserva: conserva simulación anterior y restaura selector. Contexto y kernels no se recompilan. `clFinish` completa trabajo previo; se crean cuatro buffers con `CL_MEM_COPY_HOST_PTR`. Si falla una reserva, se liberan las nuevas reservas parciales. Después de éxito se liberan los cuatro anteriores con `clReleaseMemObject` y se intercambian los handles. Es un reset transaccional: aumenta brevemente la memoria reservada para evitar destruir el estado válido ante un fallo. Los argumentos de los kernels se vuelven a establecer en el siguiente paso, como ya ocurría.

Las listas explícitas del archivo conservan su prefijo: se truncan al reducir N y se completan con y_eq/velocidad cero al aumentar N. Los patrones se regeneran con el nuevo N. Las masas fijas sobrescriben y=v=0. Un modo modal con y_eq distinto de cero y extremos fijos se rechaza con mensaje, igual que en el parser. Para N=2 fijo no queda ninguna masa móvil y ambas quedan ancladas. El reinicio reanuda la simulación.

## Dibujo e input del menú

`drawMenu()` usa `glPushAttrib/glPopAttrib` y guarda/restaura ambas pilas de matrices. Dibuja un rectángulo en coordenadas de píxel:

```cpp
glMatrixMode(GL_PROJECTION);
glPushMatrix(); glLoadIdentity();
glOrtho(0, c.width, 0, c.height, -1, 1);
glMatrixMode(GL_MODELVIEW);
glPushMatrix(); glLoadIdentity();
glColor3f(.10f, .14f, .20f);
glBegin(GL_QUADS);
// Cuatro vértices del panel con glVertex2i.
glEnd();
// glutBitmapCharacter: N, [-]/[+], (X) Fijos/(X) Libres, Reiniciar.
glPopMatrix();
glMatrixMode(GL_PROJECTION); glPopMatrix();
glMatrixMode(GL_MODELVIEW);
```

La escena conserva sus geometrías; `geometry()` llama `sceneProjection()`, `glColor3fv`, `glBegin`, `glVertex2f` y `glEnd`. También se reemplazó `glWindowPos2i`, que no pertenece a GL 1.2, por `glRasterPos2i` bajo proyección ortográfica de píxeles.

Flecha arriba aumenta N y abajo lo reduce, limitado a 2..200; F alterna el borde y R solicita reset. El panel también responde a click izquierdo. Los clicks dentro del panel se consumen antes del picking; no pueden enganchar una masa. El panel se dibuja después de la captura MP4 para no introducir controles en el video; el HUD conserva su opción independiente de grabación.

Orden: consumir reset → física → limpiar GL_BACK → escena con glOrtho → HUD/captura según opciones → panel → restaurar matrices de escena → glutSwapBuffers. Se mantiene vsync GLX opcional y doble buffer existentes.

## Kernel y significado de borde

Los kernels ya aceptaban `int first, int last` mediante `clSetKernelArg`; el menú los establece ambos al valor de `params.extremos_fijos`. No es necesario añadir recompilación ni un tercer flag redundante. El fragmento vigente de drift es:

```c
if ((i == 0 && first) || (i == n-1 && last)) {
    z[i] = 0;
    h[i] = 0;
    return;
}
```

Kick impone igualmente v=0. Para libres la aceleración omite vecinos inexistentes:

```c
float f = -k1 * (y[i] - eq);
if (i > 0)   f -= k * (y[i] - y[i-1]);
if (i+1 < n) f -= k * (y[i] - y[i+1]);
```

Esto equivale a usar el propio y[i] como vecino exterior. Para fijos se anclan las masas extremas, conforme al objetivo físico. Usar sólo un vecino exterior cero sin anclar la masa modelaría otra condición (resorte a pared con extremo móvil). No se cambió la ecuación ni Verlet. Extremos fijos permiten modos estacionarios, pero un pulso sigue siendo superposición de modos; extremos libres también admiten modos y reflexiones, no son bordes absorbentes.

## Picking con GLU

La función `screenToWorld` restaura las matrices de escena explícitamente, consulta modelview/proyección/viewport y aplica:

```cpp
GLdouble model[16], projection[16], px, py, pz;
GLint viewport[4];
glGetDoublev(GL_MODELVIEW_MATRIX, model);
glGetDoublev(GL_PROJECTION_MATRIX, projection);
glGetIntegerv(GL_VIEWPORT, viewport);
if (!gluUnProject(x, viewport[3]-1-y, .5,
                  model, projection, viewport, &px, &py, &pz))
    return false;
wx = float(px);
wy = float(py + c.L0);
```

Con glOrtho(...,-1,1) el plano z=0 corresponde a profundidad 0.5; no se necesita leer profundidad ni usar picking por color. `+L0` convierte posición dibujada a coordenada física y. Se mantiene el criterio existente de seleccionar la masa móvil más cercana y arrastrar sólo en y.

## Compilación y verificación

```bash
sudo apt install freeglut3-dev libgl-dev libglu1-mesa-dev
cmake -S . -B build
cmake --build build -j
./build/osciladores parameters/ejemplo.cfg
```

Línea de enlace:

```cmake
target_link_libraries(osciladores PRIVATE
    OpenCL::OpenCL OpenGL::GL OpenGL::GLU GLUT::GLUT)
```

No hacen falta ImGui, GLEW ni glad. El contexto solicitado es 1.2. Se verificaron compilación sin advertencias, parser, pruebas físicas, 60 cambios N/borde, dibujo del menú, round-trip GLU y captura H.264. La auditoría de símbolos del ejecutable no muestra funciones modernas de renderizado. La extensión GLX de control de vsync pertenece a la gestión de ventana; si no está disponible se informa y se continúa.

Referencia de GLU: [Khronos: transformaciones y gluUnProject](https://wikis.khronos.org/opengl/Viewing_and_Transformations).


## Ampliación: tiempo y extremos mixtos

El estado `SimParams` ahora tiene `primero_fijo`, `ultimo_fijo` e `indefinido` en lugar de un único flag de bordes. `applyReset` pasa cada extremo por separado al solver existente. El panel y F incluyen el caso primero libre / último fijo. I alterna los límites de tiempo/pasos sin resetear posiciones. El ejemplo usa tiempo_total=0. Las pruebas cubren límite de 60 s, ausencia de límite y evolución con primer extremo móvil y último anclado.
