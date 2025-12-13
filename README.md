# PRACTICA_RAZONAMIENTO

## JSON de DATOS para el LLM compatible con OpenAI
```
{
  "model": "gpt-4.1-mini",
  "messages": [
    {
      "role": "system",
      "content": [
        { "type": "text", "text": "Eres un agente de navegación. Devuelve SOLO JSON de acción." }
      ]
    },
    {
      "role": "user",
      "content": [
        { "type": "text", "text": "<STATE+SENSORS TEXT>" },
        { "type": "image_url", "image_url": { "url": "data:image/png;base64,<MAP_PNG_BASE64>" } }
      ]
    }
  ],
  "temperature": 0.2,
  "max_output_tokens": 150
}

```

## JSON de ACCION para el LLMM compatible con OPENAI
```
{
  "action": "forward|left|right",
  "strength": 0.0,
  "reason": "texto_corto_opcional"
}

```