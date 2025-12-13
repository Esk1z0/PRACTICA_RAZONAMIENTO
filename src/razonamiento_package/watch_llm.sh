#!/bin/bash

OUTPUT_FILE="llm_current.json"

echo "Capturando /llm/input_json en tiempo real..."
echo "Archivo: $OUTPUT_FILE"
echo "Abre el archivo en VSCode para ver actualizaciones"
echo "Presiona Ctrl+C para detener"
echo ""

while true; do
    # Capturar mensaje y quitar los separadores YAML (---)
    DATA=$(ros2 topic echo /llm/input_json --once --no-arr --field data 2>/dev/null | sed '/^---$/d' | sed '/^$/d')
    
    if [ ! -z "$DATA" ]; then
        # Formatear como JSON con indentación de 2 espacios
        echo "$DATA" | jq --indent 2 '.' > "$OUTPUT_FILE" 2>/dev/null || echo "$DATA" > "$OUTPUT_FILE"
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Actualizado"
    fi
    
    sleep 0.5
done