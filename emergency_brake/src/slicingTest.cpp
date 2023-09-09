#include <stdio.h>
#include <stdlib.h>

int* obtenerRangoVector(int* vector, int longitud, int inicio, int fin, int* nuevaLongitud) {
    // Ajustar índices negativos
    if (inicio < 0) {
        inicio = longitud + inicio;
    }
    if (fin < 0) {
        fin = longitud + fin;
    }

    // Asegurarse de que los índices estén en el rango válido
    inicio = inicio < 0 ? 0 : (inicio > longitud - 1 ? longitud - 1 : inicio);
    fin = fin < 0 ? 0 : (fin > longitud - 1 ? longitud - 1 : fin);

    // Calcular la longitud del nuevo vector
    *nuevaLongitud = (inicio <= fin) ? (fin - inicio +1) : (longitud - inicio + fin+1 );

    // Crear el nuevo vector y copiar los elementos
    int* resultado = (int*)malloc(*nuevaLongitud * sizeof(int));
    int i, j = 0;
    if (inicio <= fin) {
        for (i = inicio; i <= fin; i++) {
            resultado[j++] = vector[i];
        }
    } else {
        for (i = inicio; i < longitud; i++) {
            resultado[j++] = vector[i];
        }
        for (i = 0; i <= fin; i++) {
            resultado[j++] = vector[i];
        }
    }

    return resultado;
}

int main() {
    int miVector[] = {0, 1, 2, 3, 4, 5, 6, 9, 8};
    int longitud = sizeof(miVector) / sizeof(miVector[0]);
    int inicioIndice, finIndice;
    int nuevaLongitud = 0;

    printf("Vector inicial: \n\n");
    for (int i = 0; i < longitud; i++) {
        printf("%d ", miVector[i]);
    }
    // Solicitar al usuario los índices de inicio y final
    printf("\n\n\nIngrese el índice de inicio: ");
    scanf("%d", &inicioIndice);
    printf("Ingrese el índice de fin: ");
    scanf("%d", &finIndice);

    int* resultado = obtenerRangoVector(miVector, longitud, inicioIndice, finIndice, &nuevaLongitud);

    // Imprimir el nuevo vector
    printf("Nuevo vector: ");
    for (int i = 0; i < nuevaLongitud; i++) {
        printf("%d ", resultado[i]);
    }

    // Liberar la memoria del nuevo vector
    free(resultado);

    return 0;
}
