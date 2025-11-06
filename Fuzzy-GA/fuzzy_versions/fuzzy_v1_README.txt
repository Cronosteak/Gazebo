# Versión base del sistema fuzzy (v1)

## Reglas
1. Si error_pos es lejos o error_pitch es negativo → salida: subir
2. Si error_pos es cerca y error_pitch es cero → salida: nada
3. Si error_pos es medio o error_pitch es positivo → salida: bajar

## Conjuntos de pertenencia
### error_pos
- cerca: centro=0, ancho=1
- medio: centro=2, ancho=1.5
- lejos: centro=5, ancho=1

### error_pitch
- negativo: centro=-20, ancho=10
- cero: centro=0, ancho=10
- positivo: centro=20, ancho=10

### ajuste_articular
- bajar: centro=-1.5, ancho=0.5
- nada: centro=0, ancho=0.5
- subir: centro=1.5, ancho=0.5

## Notas
- Esta versión corresponde al controlador base antes de optimización GA.
- Archivo generado automáticamente el 2025-10-30.
