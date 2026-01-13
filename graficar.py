import pandas as pd
import matplotlib.pyplot as plt

# Nombre exacto de tu archivo
ARCHIVO = 'V005.CSV'

print(f"Leyendo {ARCHIVO}...")

try:
    # 1. Cargar datos
    datos = pd.read_csv(ARCHIVO)
    
    # 2. Convertir tiempo a segundos (empezando desde 0)
    tiempo = (datos['Tiempo [ms]'] - datos['Tiempo [ms]'].iloc[0]) / 1000.0
    
    # 3. Crear la ventana con 2 gráficas (filas=2, columnas=1)
    # sharex=True hace que si haces zoom en una, la otra también se mueva
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # --- GRÁFICA DE ARRIBA: ALTURA ---
    ax1.plot(tiempo, datos['Altura Relativa [m]'], color='blue', linewidth=2)
    ax1.set_title('Gráfica 1: Altura del Cohete')
    ax1.set_ylabel('Metros')
    ax1.grid(True, linestyle='--') # Cuadrícula punteada
    
    # Marcar el punto máximo (Apogeo) con un punto rojo
    altura_max = datos['Altura Relativa [m]'].max()
    ax1.axhline(altura_max, color='red', linestyle=':', alpha=0.5)
    ax1.text(tiempo.iloc[0], altura_max, f' Apogeo: {altura_max} m', color='red', va='bottom')

    # --- GRÁFICA DE ABAJO: ACELERACIÓN ---
    # Graficamos los 3 ejes para ver todo el movimiento
    ax2.plot(tiempo, datos['AccX [m/s^2]'], label='X', alpha=0.6)
    ax2.plot(tiempo, datos['AccY [m/s^2]'], label='Y', alpha=0.6)
    ax2.plot(tiempo, datos['AccZ [m/s^2]'], label='Z (Vertical)', color='black', linewidth=1.5)
    
    ax2.set_title('Gráfica 2: Aceleración (Fuerzas G)')
    ax2.set_ylabel('m/s²')
    ax2.set_xlabel('Tiempo (Segundos)')
    ax2.legend() # Muestra las etiquetas X, Y, Z
    ax2.grid(True, linestyle='--')

    # Ajustar espacio y mostrar
    print("¡Listo! Abriendo gráficas...")
    plt.tight_layout()
    plt.show()

except FileNotFoundError:
    print(f"ERROR: No encuentro el archivo {ARCHIVO}. Revisa el nombre.")
except Exception as e:
    print(f"Error inesperado: {e}")