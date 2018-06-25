#include <Python.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

/* reads from keypress, doesn't echo */
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

/* reads from keypress, echoes */
int getche(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

static PyObject *getch_getche(PyObject *self, PyObject *args)
{
	int ok = PyArg_ParseTuple(args, "");
	char c = getche();
	return PyUnicode_FromFormat("%c", c);
}

static PyObject *getch_getch(PyObject *self, PyObject *args)
{
	int ok = PyArg_ParseTuple(args, "");
	char c = getch();
	return PyUnicode_FromFormat("%c", c);
}

static PyMethodDef GetchMethods[] = {
	{"getch",  getch_getch, METH_VARARGS, "Reads a character from standard input, not echoing it."},
	{"getche", getch_getche, METH_VARARGS, "Reads a character from standard input, displaying it on the screen"},
	{NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef getchmodule = {
   PyModuleDef_HEAD_INIT,
   "getch",   /* name of module */
   NULL, /* module documentation, may be NULL */
   -1,       /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
   GetchMethods
};

PyMODINIT_FUNC PyInit_getch(void)
{
	return PyModule_Create(&getchmodule);
}
