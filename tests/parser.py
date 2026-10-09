"""Parser rejection tests: no GPU/display required. Run with executable path."""
import pathlib, subprocess, sys, tempfile
exe = str(pathlib.Path(sys.argv[1]).resolve())
cases = [('N=1\n', True), ('N=0\n', False), ('dt=nan\n', False),
         ('m=-1\n', False), ('N=3\ny_inicial=[1,2]\n', False),
         ('N=2\nN=3\n', False), ('extremo_superior_fijo=yes\n', False),
         ('desconocido=1\n', False), ('N=1\nextremo_inferior_fijo=true\npatron=modo_normal\n', False)]
with tempfile.TemporaryDirectory() as directory:
    path = pathlib.Path(directory) / 'case.cfg'
    for contents, valid in cases:
        path.write_text(contents)
        result = subprocess.run([exe, str(path), '--validate'], capture_output=True)
        assert (result.returncode == 0) == valid, (contents, result.stderr)
print('Parser: 9 casos OK')
