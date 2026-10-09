# Osciladores verticales acoplados — OpenCL + OpenGL

Simulador educativo de ondas mecánicas en una cadena de masas suspendidas y acopladas por resortes, inspirado en Crawford. C++17 gestiona la ventana, OpenCL calcula la dinámica y OpenGL 1.2 dibuja con pipeline fijo. Incluye arrastre con mouse, excitación sinusoidal, bordes configurables, HUD y exportación MP4.

## Archivos y compilación

```text
CMakeLists.txt
include/simulation.hpp       interfaces, parámetros y recursos
src/config.cpp               parser, condiciones iniciales y energía
src/dynamics.cpp             dispositivo, memoria y ejecución OpenCL
src/main.cpp                 ventana GL 1.2, HUD, menú, reloj y mouse
src/recorder.cpp             captura RGB y proceso ffmpeg
kernels/verlet.cl            drift y kick de velocity Verlet
parameters/ejemplo.cfg       configuración comentada
build/                      ejecutable y archivos de CMake
```

Dependencias: compilador C++17, CMake >=3.16, OpenCL ICD y runtime del fabricante (o PoCL para CPU), OpenGL 1.2 con pipeline fijo y GLU, FreeGLUT; ffmpeg con encoder libx264 para MP4. Implementación host/ventana/captura orientada a Linux/POSIX (fork, pipe, GLX); kernels OpenCL 1.2 independientes del fabricante. El port a Windows requiere adaptar captura y carga de funciones GL; no se afirma portabilidad automática del host a todos los sistemas operativos.

En Debian/Ubuntu:

```bash
sudo apt install build-essential cmake ocl-icd-opencl-dev freeglut3-dev libgl-dev libglu1-mesa-dev ffmpeg
# Si no hay runtime GPU y se desea ejecutar en CPU:
sudo apt install pocl-opencl-icd
cmake -S . -B build
cmake --build build -j
./build/osciladores parameters/ejemplo.cfg
./build/osciladores parameters/ejemplo.cfg --validate
# Headless requiere limite temporal: definir n_pasos o tiempo_total > 0.
# ./build/osciladores parameters/ejemplo.cfg --headless
ctest --test-dir build --output-on-failure
```

`--validate` valida sin requerir GPU ni ventana. `--headless` ejecuta física OpenCL sin OpenGL hasta n_pasos o tiempo_total; las pruebas de CTest requieren dispositivo OpenCL. El programa imprime dispositivo y errores del compilador de kernels. Error -1001 significa que no se encontró una plataforma accesible. En entornos aislados puede ser necesario permitir acceso al dispositivo.

Los recursos se localizan con la ruta de fuente incorporada por CMake, por lo que el ejecutable puede invocarse desde otra carpeta mientras se conserve el proyecto. Los videos se guardan relativos al directorio de ejecución. No se utiliza libav directamente ni se descargan dependencias durante la ejecución.

## Física y relación con Crawford

Cada coordenada y_i es desplazamiento vertical en metros, con x_i=i*x0 invariable. El resorte al soporte restaura a y_eq. La gravedad se considera absorbida en ese equilibrio; no se agrega mg. El dibujo pone la masa en (x_i,-L0+y_i), con soporte en Y=0. Es un modelo lineal de osciladores acoplados inspirado en el desarrollo de modos normales y propagación del libro de Frank S. Crawford Jr., *Waves*, Berkeley Physics Course, volumen 3; no una reproducción de un aparato o figura concreta. [Texto y notas del autor](https://physics.csuchico.edu/web/pdf_books/Crawford_Waves_Vol_3/Crawford_all.pdf).

La aceleración es exactamente:

```text
m*a_i = -k1*(y_i-y_eq) - k*(y_i-y_(i-1)) - k*(y_i-y_(i+1))
```

Se omiten vecinos inexistentes. Los extremos fijos son las MASAS de índice 0 y N-1, impuestas a y=v=0; no son masas ficticias fuera de la cadena. Los vecinos sí sienten su acoplamiento a esas masas. Los nombres superior/inferior del archivo se mantienen aunque la cadena de columnas se dibuja horizontalmente.

Los acopladores dibujados como resortes oblicuos representan fuerzas lineales en diferencias verticales. No se calcula la fuerza geométrica de un resorte de Hooke horizontal de longitud sqrt(x0²+Δy²): daría un modelo diferente, generalmente no lineal. L0 y L0_acople controlan dibujo/color, no la aceleración. El color mide diferencia de longitud geométrica respecto a la natural visual. No hay movimiento en x. Las oscilaciones son verticales, transversales respecto de la dirección horizontal de propagación; «longitudinal» aquí se refiere al movimiento a lo largo del resorte local vertical.

```text
Ec = sum_i (m/2)*v_i²
Ep = sum_i (k1/2)*(y_i-y_eq)² + sum_(i=1..N-1) (k/2)*(y_i-y_(i-1))²
```

En dinámica libre la energía total presenta error oscilatorio acotado, propio de Verlet. Durante arrastre el usuario hace trabajo y cambia la energía. Los acoplamientos intercambian momento entre vecinos; el momento total de las masas NO se conserva con k1>0 o anclajes, porque actúan fuerzas externas. Para extremos libres y k1=0 sí se conserva. En cadena libre uniforme la dispersión es omega² = (k1+4*k*sin²(q*x0/2))/m; k1 agrega una frecuencia de corte.

## Parámetros y experimentos

El formato es `clave = valor`, con comentarios `#`, booleanos true/false o True/False, cadenas opcionalmente entre comillas y listas `[a,b,c]`. Sigue el estilo de asignaciones de parametros.py, pero es un parser de datos: no acepta fórmulas ni ejecuta Python. Rechaza claves desconocidas/duplicadas, no finitos y listas de longitud incorrecta. Todos los campos editables están en el ejemplo; cambios de archivo se leen al iniciar, R reinicia con N y borde elegidos en el menú; los demás parámetros se toman del archivo cargado al iniciar.

- Pulso: `patron=pulso_extremo`, `amplitud=0.7`, velocidades omitidas. Excita la primera masa libre; con extremo 0 fijo excita índice 1. Si no hay masa libre no hay pulso.
- Perfil arbitrario: `patron=arbitrario`, `y_inicial=[...]` con exactamente N valores absolutos en metros. `v_inicial=[...]` usa m/s. Los extremos fijos sobrescriben sus valores a cero.
- Modos: `patron=modo_normal`, `modo=1..M` donde M son masas libres. Fijo/fijo: seno; libre/libre: coseno, modo 1 uniforme; mixto: coseno con reflexión. Con extremos fijos se requiere y_eq=0 para que el perfil modal coincida con el equilibrio. Para y_eq distinto de cero y extremos anclados, usar perfil arbitrario y considerar el equilibrio estático no uniforme.
- `patron=cero` coloca masas móviles en y_eq, v=0 (fijas en 0). Con y_eq distinto de cero y extremos fijos puede haber movimiento por la incompatibilidad de equilibrios.
- N=2 muestra intercambio entre dos osciladores; N=64 o 128 permite ver propagación y reflexión. Aumentar k acelera transferencia; k=0 desacopla columnas. k1=0 y extremos libres aproximan cadena sin restauración local. Reducir dt cuando se incrementa k o se disminuye m.
- `extremo_superior_fijo=true` y `extremo_inferior_fijo=true` muestran reflexiones en anclajes; false/false deja extremos libres. N=1 es válido.
- `tiempo_total=0`, `n_pasos=0`: ejecución ilimitada. Si ambos son positivos la ventana pausa al primero alcanzado. Con dt que no divide tiempo_total se detiene en el último paso completo anterior.
- `escala` ajusta zoom; radio y colores permiten mejorar visibilidad. La cámara usa x0, N, L0 y amplitud inicial; arrastres muy grandes pueden salir del encuadre (no se hace zoom automático durante interacción).

## Interacción, menú y video

El panel en pantalla permite cambiar N (2..200), elegir extremos fijos/libres y reiniciar; también se usan flechas arriba/abajo y F. Click derecho abre el menú de acciones; teclado: Espacio pausa/reanuda, R reinicia, E habilita/deshabilita edición, H muestra/oculta HUD, A muestra/oculta animación, G inicia/finaliza MP4, B cambia HUD de grabación, Esc sale. Al ocultar la animación la física continúa.

Click izquierdo cerca de una masa móvil la selecciona (rojo); arrastrar cambia sólo su y, las otras evolucionan. Las masas ancladas no pueden arrastrarse. Soltar conserva la velocidad del último segmento del cursor, o cero con `soltar_velocidad_cero=true`. Si el cursor se detuvo más de 80 ms, se suelta con cero para evitar velocidad residual antigua. Funciona también en pausa. No se limita velocidad: movimientos bruscos pueden inyectar mucha energía.

Grabación: pipe POSIX entrega RGB24 desde GL_BACK antes de swap; ffmpeg invierte filas, codifica H.264/yuv420p y finaliza el contenedor al cerrar stdin. No se interpolan fotogramas perdidos: se duplican para sostener la duración real a fps configurado. No hay widgets recordatorios ni indicadores de grabación en el framebuffer; el menú nativo es una superficie separada, no capturada. HUD_grabacion controla exclusivamente el HUD del archivo, incluso si en pantalla está oculto; mostrar_HUD controla la ventana. Cerrar, salir o redimensionar finaliza la grabación. Las dimensiones impares asignadas por el gestor de ventanas se recortan en un pixel para MP4. ffmpeg informa errores en terminal; errores de pipe no abortan por SIGPIPE.

## Integración y sincronización

Velocity Verlet usa v_half=v+a(y)*dt/2, y_next=y+v_half*dt, v_next=v_half+a(y_next)*dt/2. Dos kernels en cola in-order constituyen una barrera global: no se intenta sincronizar grupos de trabajo dentro de un único kernel. Cuatro buffers O(N), float32 (posiciones ping-pong, velocidades, velocidades intermedias); todos permanecen en OpenCL entre pasos. La interacción escribe sólo una posición y velocidad. El índice arrastrado es una restricción externa en ambos kernels. Energía calculada en host con acumulación double.

La cota conservadora omega_max=sqrt((k1+4*k)/m) permite advertir cuando dt>=2/omega_max; para exactitud conviene usar dt mucho menor. Es simpléctico para el sistema autónomo libre, no durante imposición del mouse. Los extremos libres pueden tener omega=0 si k1=k=0; no hay entonces límite de estabilidad oscilatoria.

Un reloj monotónico alimenta un acumulador de tiempo real. dt no cambia; se ejecutan los pasos necesarios hasta max_pasos_frame, conservando sólo la fracción menor que dt al saturar. El HUD informa el tiempo descartado: bajo carga extrema la física se ralentiza respecto del reloj real. Pausa borra acumulador. Se interpola entre los dos últimos estados físicos para renderizar sin saltos (latencia de un paso); HUD de energía usa el estado físico actual, no la interpolación visual.

Lecturas bloqueantes CL_TRUE garantizan datos completos antes de dibujar con glBegin/glEnd. Se evita interoperabilidad CL/GL obligatoria: permite GPU/CPU OpenCL distinta del renderizador y evita carreras acquire/release. La [referencia de Khronos](https://registry.khronos.org/OpenCL/specs/unified/refpages/man/html/clEnqueueReadBuffer.html) describe esta garantía. Copias O(N) por frame son razonables para una cadena educativa; no se promete máxima eficiencia para millones de masas.

FreeGLUT usa doble buffer; se dibuja GL_BACK y se intercambia una vez por frame. Se solicita vsync mediante GLX_EXT_swap_control si está disponible; la tasa configurada es objetivo, no garantía de 60 Hz. El driver/compositor puede imponer otra sincronización. Grabación usa glReadPixels síncrono y pipe con contrapresión: es deliberadamente sencillo y puede reducir FPS; las duplicaciones sostienen la duración del video pero no recuperan suavidad perdida. Para captura de alta resolución sería conveniente PBOs y un hilo de codificación.

## Validación

`--self-test` verifica en OpenCL: extremos Dirichlet, conservación de energía de modo normal, oscilador aislado frente a coseno analítico, conservación del momento con k1=0 y restricción de arrastre. Probado en NVIDIA RTX A4000: máximo error relativo de energía observado ≈9.3e-6 en 10000 pasos del caso modal. La compilación C++17 pasa con -Wall -Wextra -Wpedantic.

`--render-test` abre la ventana, graba 20 frames de prueba y sale automáticamente.


## Cambios del menú GL 1.2

Ver [CAMBIOS_MENU.md](CAMBIOS_MENU.md) para archivos modificados, fragmentos de código, política de reinicialización y compilación. No hay shaders, VAOs, VBOs ni dependencia de ImGui, GLEW o glad. Se conservan las lecturas bloqueantes OpenCL, el timestep fijo, el acumulador, doble buffer y solicitud de vsync. La extensión GLX de vsync es una operación de ventana opcional, no un requisito de la escena.

Pruebas adicionales: 60 reallocaciones alternando N=2/N=200 y bordes, reset del estado y evolución sin valores no finitos. `--render-test` también alterna N/borde entre frames, comprueba GLU project/unproject y errores GL, y guarda `build/menu_preview.ppm`. La revisión gráfica se realizó en NVIDIA con contexto compatible; no se ha probado físicamente una GPU antigua con OpenGL 1.2.


## Ejecución indefinida y borde mixto

El ejemplo ahora usa `tiempo_total=0` y `n_pasos=0`, por lo que sigue hasta cerrar o pausar. Antes se detenía al llegar a 60 s por `tiempo_total=60`. El panel incluye **Tiempo indefinido** (tecla **I**): ignora ambos límites sin reiniciar el estado y, si estaba detenido por el límite, reanuda. Desactivarlo restaura los límites del archivo; si ya se superaron, volverá a pausar. Con ambos límites del archivo en cero, desactivarlo también deja la ejecución sin límite.

**F** recorre libres → fijos → primero libre/último fijo. El panel tiene además un botón directo para **Libre primero / fijo último**. Cambiar borde reinicia la simulación, cancela el arrastre y conserva la elección de tiempo indefinido. Para iniciar con ese borde desde archivo:

```text
extremo_superior_fijo = false
extremo_inferior_fijo = true
```

Esto permite explorar modos y reflexiones de una cadena con un extremo móvil y otro anclado. Es una analogía educativa para un brazo de un diapasón; el modelo sigue siendo una cadena de masas con fuerzas lineales, no una simulación de flexión de sus dos brazos ni de su geometría real.


## Primera masa forzada y formación de patrones estacionarios

En el panel, **Primera masa forzada** o **O** activa una fuerza externa `F(t)=F0*sin(phi)` sobre índice 0. **F0** se cambia con botones +/- (0.1 N) o **[ / ]**; **f** con botones +/- (0.01 Hz) o **, / .**. La masa forzada se muestra amarilla. Ambos valores pueden ajustarse durante la evolución, sin reset; se conserva la fase y se actualiza su velocidad angular a `2*pi*f`. Al reiniciar, la fase vuelve a cero. Durante pausa la fase no avanza. Desactivar la fuerza deja la masa evolucionando libremente.

Para activar la fuerza, el primer extremo debe estar libre. Si estaba fijo, O lo libera y reinicia como cualquier cambio de borde, dejando el último extremo como estaba. Elegir un borde con primera masa fija desactiva el forzado. Arrastrar la primera masa tiene prioridad sobre la fuerza mientras dure el arrastre. El archivo con forzado activo y primer extremo fijo se rechaza para evitar una configuración contradictoria.

Parámetros: `primer_oscilador_forzado` (bool), `amplitud_fuerza` (N, >=0), `frecuencia_fuerza` (Hz, >0). No confundir `amplitud_fuerza` con `amplitud`, que controla desplazamiento inicial en metros. Velocity Verlet evalúa la fuerza en los tiempos de ambos medios kicks; los kernels reciben los dos valores calculados a partir de una fase acumulada en double.

Experimento preparado:

```bash
./build/osciladores parameters/onda_forzada.cfg
```

Tiene N=20, primera masa libre, última fija, todas las posiciones/velocidades inicialmente cero, ejecución ilimitada y frecuencia 0.51330 Hz, correspondiente al tercer modo. Para M=N-1 masas móviles, los modos de este borde tienen:

```text
f_j = sqrt((k1 + 4*k*sin²((j-1/2)*pi/(2*M+1)))/m)/(2*pi), j=1..M
```

El forzado se transmite por los acoplamientos y la reflexión en el extremo fijo permite observar un patrón con nodos y vientres. El modelo no tiene amortiguamiento: persisten transitorios y, en resonancia, la amplitud crece; no hay una respuesta estacionaria de amplitud finita a la que deba converger. La energía ya no se conserva porque la fuerza externa realiza trabajo. Puede reducirse F0 o reiniciar para mantener el patrón dentro de cámara.

Prueba adicional: oscilador aislado forzado comparado con solución analítica, y cambio de amplitud/frecuencia conservando la iteración actual.


## Visibilidad y ampliación vertical

H recorre: todo visible → ocultar HUD (incluido mini-gráfico) → ocultar también panel de botones → mostrar sólo información → mostrar también botones. El panel oculto no intercepta clicks; los atajos siguen activos. La opción HUD_grabacion continúa siendo independiente de la visibilidad en pantalla.

**Escala Y** en el panel, o **+ / -**, multiplica/divide por 1.25 la amplitud vertical mostrada. `escala_y=1` conserva la representación original, `escala_y=5` muestra desplazamientos cinco veces mayores. No modifica posiciones, velocidades, fuerzas, dt ni escala horizontal. El equilibrio se dibuja en la misma posición; sólo se amplifica `y-y_eq`, para que los resortes no crezcan por el factor y oculten la onda. El picking y arrastre aplican la transformación inversa. R conserva la escala elegida; es configurable al iniciar en el archivo, rango 0.01..1000.


## Desarrollo y GitHub

El repositorio incluye sólo fuentes, configuraciones de ejemplo, documentación y pruebas. `.gitignore` excluye ejecutables, caches, grabaciones y configuraciones locales de editores. El proyecto se compila desde un checkout nuevo siguiendo los comandos anteriores; no depende de los otros simuladores de la carpeta original.

El workflow `.github/workflows/ci.yml` compila en Linux y ejecuta CTest en cada push y pull request. Usa OpenCL en CPU mediante PoCL para no depender de una GPU NVIDIA en el servidor. Comprueba parser, archivos de ejemplo y dinámica; el renderizado y la captura MP4 se prueban localmente mediante `--render-test`, ya que requieren ventana y ffmpeg. Los ejemplos se mantienen como fueron configurados por el autor: `ejemplo.cfg` tiene amplitud inicial cero, por lo que permanece en reposo hasta arrastrar una masa o activar la fuerza. Para una demostración inmediata, usar `onda_forzada.cfg`.

`CAMBIOS_MENU.md` conserva notas históricas de implementación. La guía actual de uso es este README.
