#!/usr/bin/env python3
import ollama
import os
import sys

def main():
    print("🤖 Ollama Bağlantı Testi (A4)")
    print("--------------------------------")
    
    host = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434")
    print(f"Hedef LLM Sunucusu: {host}")

    try:
        # Doğrudan Client tanımlıyoruz
        client = ollama.Client(host=host)
        
        # 1. Modelleri listele
        models = client.list()
        
        if not models.get('models'):
            print("\n❌ Ollama ayakta ancak yüklü model yok.")
            print("Lütfen bir model indirin: docker compose exec ollama ollama pull codellama:7b-code")
            sys.exit(1)
            
        print("\n📦 Yüklü Modeller:")
        for m in models['models']:
            print(f"   - {m['name']}")
            
        test_model = models['models'][0]['name']
        
        # 2. Üretim (Inference) yap
        print(f"\n🧠 Seçilen Model: {test_model}")
        print("⏳ İlk Inference testi yapılıyor...")
        
        response = client.generate(
            model=test_model,
            prompt="A4 projesi (Adversarial Test) başlıyor, bana Türkçe olarak 'Hazırım Kaptan' de.",
            stream=False
        )
        
        print("\n✅ Yanıt alındı:")
        print(f"   \"{response['response']}\"\n")
        print("🎉 Test başarılı! (Python Sandbox -> Ollama bağlantısı çalışıyor)")
        
    except Exception as e:
        print(f"\n❌ Ollama'ya bağlanırken hata oluştu: {e}")
        print("   Container B (Ollama) ayakta mı kontrol edin.")
        sys.exit(1)

if __name__ == "__main__":
    main()
