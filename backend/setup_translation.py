#!/usr/bin/env python3
"""
Setup script for translation models and language packages.
This script installs the required language packages for Argos Translate
and downloads MarianMT models for fallback translation.
"""
import os
import subprocess
import sys
from pathlib import Path

def install_argos_language_packs():
    """Install language packs for Argos Translate"""
    try:
        import argostranslate
        from argostranslate import translate

        print("Installing Argos Translate language packages...")

        # Update package index
        translate.update_available_packages()

        # Get available packages
        available_packages = translate.get_available_packages()

        # Install common language pairs (English as source or target)
        supported_languages = ['es', 'fr', 'de', 'it', 'pt', 'ru', 'zh', 'ja', 'ar', 'hi']

        for package in available_packages:
            if package.from_code in ['en'] and package.to_code in supported_languages:
                print(f"Installing {package.from_code} to {package.to_code} package...")
                package.install()
            elif package.from_code in supported_languages and package.to_code in ['en']:
                print(f"Installing {package.from_code} to {package.to_code} package...")
                package.install()

        print("Argos Translate language packages installation completed.")

    except ImportError:
        print("Argos Translate not available, skipping language package installation.")
    except Exception as e:
        print(f"Error installing Argos Translate packages: {e}")

def download_marianmt_models():
    """Download MarianMT models for fallback translation"""
    try:
        from transformers import AutoTokenizer, AutoModelForSeq2SeqLM

        print("Downloading MarianMT models for common language pairs...")

        # Common language pairs for MarianMT
        model_pairs = [
            'Helsinki-NLP/opus-mt-en-es',  # English to Spanish
            'Helsinki-NLP/opus-mt-es-en',  # Spanish to English
            'Helsinki-NLP/opus-mt-en-fr',  # English to French
            'Helsinki-NLP/opus-mt-fr-en',  # French to English
            'Helsinki-NLP/opus-mt-en-de',  # English to German
            'Helsinki-NLP/opus-mt-de-en',  # German to English
            'Helsinki-NLP/opus-mt-en-zh',  # English to Chinese
            'Helsinki-NLP/opus-mt-zh-en',  # Chinese to English
        ]

        for model_name in model_pairs:
            print(f"Downloading {model_name}...")
            try:
                tokenizer = AutoTokenizer.from_pretrained(model_name)
                model = AutoModelForSeq2SeqLM.from_pretrained(model_name)
                print(f"Successfully downloaded {model_name}")
            except Exception as e:
                print(f"Error downloading {model_name}: {e}")

        print("MarianMT models download completed.")

    except ImportError:
        print("Transformers not available, skipping MarianMT models download.")
    except Exception as e:
        print(f"Error downloading MarianMT models: {e}")

def main():
    print("Setting up translation models and language packages...")

    install_argos_language_packs()
    download_marianmt_models()

    print("Translation model setup completed.")

if __name__ == "__main__":
    main()