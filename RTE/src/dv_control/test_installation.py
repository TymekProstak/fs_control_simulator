#!/usr/bin/env python3
"""Test instalacji środowiska MPC"""

import sys
import os

def test_imports():
    print("=" * 50)
    print("TEST IMPORTÓW")
    print("=" * 50)
    
    packages = [
        ('numpy', 'NumPy'),
        ('scipy', 'SciPy'),
        ('casadi', 'CasADi'),
        ('matplotlib', 'Matplotlib'),
        ('acados_template', 'ACADOS Template')
    ]
    
    failed = []
    for package, name in packages:
        try:
            __import__(package)
            print(f"✓ {name}")
        except ImportError as e:
            print(f"✗ {name}: {e}")
            failed.append(name)
    
    if failed:
        print(f"\n❌ Błąd: Nie udało się zaimportować: {', '.join(failed)}")
        return False
    else:
        print("\n✅ Wszystkie pakiety zainstalowane poprawnie!")
        return True

def test_acados_env():
    print("\n" + "=" * 50)
    print("TEST ZMIENNYCH ŚRODOWISKOWYCH ACADOS")
    print("=" * 50)
    
    acados_dir = os.environ.get('ACADOS_SOURCE_DIR')
    if acados_dir:
        print(f"✓ ACADOS_SOURCE_DIR: {acados_dir}")
        
        lib_path = os.path.join(acados_dir, 'install', 'lib', 'libacados.so')
        if os.path.exists(lib_path):
            print(f"✓ Biblioteka ACADOS znaleziona: {lib_path}")
        else:
            print(f"✗ Biblioteka ACADOS NIE znaleziona: {lib_path}")
            return False
    else:
        print("✗ ACADOS_SOURCE_DIR nie ustawione!")
        return False
    
    return True

if __name__ == '__main__':
    success = test_imports() and test_acados_env()
    sys.exit(0 if success else 1)
