# Symulacja DV – Planning + SLAM + Kontrola (LEM)

Symulacja obejmuje pełny pipeline **planowania**, **SLAM-u** oraz **kontroli** pojazdu DV. Oparta jest o uproszczony model fizyki auta, wystarczający na potrzeby weryfikacji algorytmów projektu.

System uwzględnia:
- szum na czujnikach (wizja, DRI),
- opóźnienia w przepływie danych pomiędzy:
  - wizją,
  - `dv_board`,
  - aktuatorami,
  - resztą systemu.

  Fizyka opiera się na kwazistatycznym modelu zawieszenia oraz pseudo-modelu opony MF6.1 w stanie ustalonym z 1-rzędową relaksacją. W niskich prędkościach model Magic Formula jest zastępowany modelem pseudo-Coulombowskim.

  Wizja zawiera szum w funkcji odległości opracowany na podstawie artykułu dotyczącego kamery ZED2i oraz heurystykę określającą prawdopodobieństwo braku obserwacji pachołka w danej klatce, modelowaną funkcją eksponencjalną względem odległości( w obecniej wersji wykomentowane)

  Opóźnienia na wizji oraz układzie `dv_board` zostały odtworzone na podstawie konfiguracji sprzętowej, częstotliwości pętli odczytu/wysyłania oraz rzeczywistych czasów obliczeń analitycznych z logów.

  Model INS obsługuje dwie opcje: wartość rzeczywistą (Ground Truth) z dodatkiem szumu Gaussa (gdzie błędy dobierane są heurystycznie, dążąc do zbliżenia ich jakości do modułów Ublox F9P GPS RTK) lub filtr EKF (Kalman) w trybie 2D. Filtr jest zasilany przez dryfujące z czasem IMU i dane GPS (orientacja, prędkość, pozycja) sparametryzowane wartościami błędów wyciągniętymi bezpośrednio z not aplikacyjnych modułu Ublox F9P (dane zapewne dobre) oraz danymi od Pana Grzegorza (dane o wątpliwej pewności).

## Konfiguracja środowiska (LEM vs RTE)
RTE póki co wywalone

## Argumenty uruchomieniowe (Launch Options)

Symulacja posiada dwie główne opcje konfiguracyjne:

| Argument | Wartość domyślna | Opis |
| :--- | :--- | :--- |
| `ins_mode` | `"gauss"` | Określa tryb symulacji DRI.<br>- `kalman`: symulacja filtra Kalmana karmionego IMU i GPS RTK.<br>- `gauss`: Ground Truth + szum biały + bias. |
| `sim_time` | `"-1"` | Czas trwania symulacji w sekundach.<br>- `-1`: nieskończoność.<br>- `int`: liczba sekund. |

Ponadto system można uruchomić w dwóch dodatkowych konfiguracjach testowych w celu wyizolowania konkretnych modułów:
   - `roslaunch lem_simulator no_pp_test.launch ins_mode:="kalman"` – w tym scenariuszu ścieżka dostarczana do układu kontroli pochodzi bezpośrednio z pliku, omijając w pełni pipeline SLAM -> PATH PLANNING.
   - `roslaunch dv_control control.launch only_low_speed:=true` – w tym trybie nie włącza się kontroler MPC ani logika dynamicznego przyspieszania i zwalniania. Domyślnie wykorzystywany jest natomiast algorytm Stanley w połączeniu z generatorem stałej, niskiej prędkości utrzymywanym przez regulator PID.

---

## Uruchamianie symulacji – standardowy launch całości

Całość można uruchomić (np. korzystając z **Foxglove**), wykonując poniższe polecenia w podanej kolejności w oddzielnych terminalach:

```bash
# 1. Start rdzenia ROS
roscore

# 2. Załadowanie opisu pojazdu
roslaunch dv_bolid_description description.launch

# 3. Uruchomienie symulatora (z opcjonalnymi argumentami) 
roslaunch lem_simulator sim.launch ins_mode:="kalman"

# 4. Uruchomienie SLAM
roslaunch dv_slam slam.launch

# 5. Uruchomienie planowania ścieżki
rosrun dv_path_planning dv_path_planning_discovery_node

# 6. Uruchomienie kontroli
# UWAGA: Przed uruchomieniem upewnij się, że zmienne środowiskowe dla acados są wyeksportowane (LD_LIBRARY_PATH),
# aby linker nie zgłaszał błędów.
# DLA LEM :  LD_LIBRARY_PATH=~/dv_ws/LEM/src/dv_control/External/acados/install/lib:$LD_LIBRARY_PATH 
roslaunch dv_control control.launch 
```

---

## Konfiguracja zależności MPC (acados)

Aby skonfigurować niezbędne zależności dla kontrolera MPC, wykonaj poniższe kroki:
**UWAGA:** Jeśli nie wykonasz tych kroków, projekt nie skompiluje się poprawnie (niezbędne pliki solvera zależą ściśle od zdefiniowanych parametrów i muszą zostać wygenerowane w czasie instalacji). Kompilacja i zależności potrafią zająć łącznie około 2 GB. Aby nie powtarzać całego procesu każdorazowo przy zmianie brancha (folder `External/` nie jest śledzony w Git), warto wcześniej przenieść/linkować kopię zbudowanego środowiska powiązanego z bazą narzędzia w inne znane i nienaruszane miejsce, powołując się w zmiennych środowiskowych na ten katalog (część instalacyjna paczki `External` dotycząca oprogramowania bazowego pozostaje uniwersalna dla tego projektu). Sam solver natomiast, znajdujący się w katalogach `mpc_solver/` i `acados_solver_generated/`, musi zostać przegenerowany z każdym momentem wprowadzania nowych ustawień z kategorii modyfikujących limity i właściwości solvera (`bounds/solver`), o czym wspomina dalsza część sekcji.

1. **Wejdź do katalogu solvera:**
   ```bash
   cd dv_control/mpc_solver
   ```

2. **Nadaj uprawnienia i zainstaluj acados lokalnie:**
   ```bash
   chmod +x install_acados_local.sh
   chmod +x run_solver_gen.sh
   ./install_acados_local.sh
   ```

3. **Ręczna instalacja `t_renderer`:**
   Automatyczne pobieranie może nie działać na starszych wersjach Ubuntu.
   * Pobierz plik ręcznie: [t_renderer v0.0.31](https://github.com/acados/tera_renderer/releases/tag/v0.0.31)
   * Skopiuj pobrany plik do: `dv_control/External/acados/bin`
   * Zmień nazwę pliku na: `t_renderer`
   * Nadaj prawa do uruchamiania:
       ```bash
       chmod +x dv_control/External/acados/bin/t_renderer
       ```

4. **Wygeneruj solver:**
   ```bash
   cd dv_control/mpc_solver
   ./run_solver_gen.sh
   ```

Jeżeli proces przebiegł poprawnie, logi w konsoli poinformują o pomyślnym wygenerowaniu solvera.

---

## Uwagi (MPC)

Zmiana parametru `mpc_horizon` lub dowolnych parametrów z kategorii `bounds` w plikach konfiguracyjnych (np. zmiana horyzontu ze 100 na inną wartość) **wymaga ponownej kompilacji solvera**.

Po każdej takiej zmianie należy uruchomić:
```bash
./run_solver_gen.sh
```

Jeśli z ciekawości ktoś chciałby zobaczyć inne kontrolery LTI Charmles/KaRacing – to pewnie nie działają. Wersje te są generalnie znacznie gorsze niż obecnie zaimplementowany LPV, ale są zgodne z treścia prac tych zespolów , które byłem w stanie znalźć w interancie. Ich kompatybilne i poprawnie skonfigurowane warianty (istnieją lokalnie) zostaną dodane do głównego drzewa przy jakieś akutalizacji całośći.

W korzeniu projektu znajduje się wyeksportowany plik JSON (np. `agh_racing_sim.json`) zawierający layout przygotowany specjalnie pod program Foxglove Studio, zalecany do wczytania tuż po odpaleniu symulacji.

W configu paczki lem_simulator na wizji jest ustawiony dwa razy mniejszy szum niż w artykule oraz zasięg 14 m zamiast 8 m, na normalnych ustawieniach jest padaka, co wskazuje na problem z obecnym stanem SLAM/potrzebą integracji z lidarem.

CZAT

