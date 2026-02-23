#!/usr/bin/env python3
"""A4 Projesi — Ollama LLM Bağlantı Testi

Container C (testrunner) ile Container B (ollama) arasındaki
HTTP API bağlantısını ve model yanıtını doğrular.
"""
import ollama
import os
import sys


def main():
    print("🤖 Ollama Bağlantı Testi (A4)")
    print("--------------------------------")

    host = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434")
    print(f"Hedef LLM Sunucusu: {host}")

    try:
        client = ollama.Client(host=host)

        # 1. Modelleri listele
        result = client.list()
        model_list = result.models if hasattr(result, 'models') else result.get('models', [])

        if not model_list:
            print("\n❌ Ollama ayakta ancak yüklü model yok.")
            print("   docker compose exec ollama ollama pull dolphin-mistral:7b")
            sys.exit(1)

        print("\n📦 Yüklü Modeller:")
        for m in model_list:
            name = m.model if hasattr(m, 'model') else m.get('name', '?')
            print(f"   - {name}")

        # İlk modeli seç
        test_model = model_list[0].model if hasattr(model_list[0], 'model') else model_list[0]['name']

        # 2. Inference yap
        print(f"\n🧠 Seçilen Model: {test_model}")
        print("⏳ İlk Inference testi yapılıyor...\n")

        response = client.generate(
            model=test_model,
            prompt="Say exactly: 'Hazırım Kaptan, A4 Adversarial Test platformu aktif!'",
            stream=False
        )

        answer = response.response if hasattr(response, 'response') else response.get('response', '')
        print(f"✅ Yanıt alındı:\n   \"{answer.strip()}\"\n")
        print("🎉 Test başarılı! (TestRunner → Ollama bağlantısı çalışıyor)")

    except Exception as e:
        print(f"\n❌ Hata: {e}")
        print("   Container B (Ollama) ayakta mı kontrol edin.")
        sys.exit(1)


if __name__ == "__main__":
    main()
