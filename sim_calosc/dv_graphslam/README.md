# Zmiany wdrożone w GraphSLAM (aktualny stan)

Ten dokument opisuje wyłącznie zakres zmian wprowadzonych w bieżącej serii poprawek. Zakładamy, że czytelnik zna wcześniejszą strukturę kodu.

## 1) Łączenie landmarków po optymalizacji

### 1.1 Kryterium kompatybilności par landmarków
Scalanie nie opiera się już tylko na odległości euklidesowej. Dla pary landmarków wymagane jest:

- zgodność typu stożka, jeśli `match_cone_types=true` (z wyjątkiem typu `unknown`),
- przejście pre-gate euklidesowego (`data_association.radius`),
- opcjonalnie (gdy `use_mahalanobis=true`) przejście gate Mahalanobisa na sumie kowariancji.

Sprawdzanie kompatybilności jest wydzielone do `areLandmarksMergeCompatible()`.

### 1.2 Fuzja stanu landmarku przy merge
Przy scaleniu wykonywana jest fuzja w postaci informacyjnej (z regularizacją diagonalną dla stabilności numerycznej):

$$
\Sigma = (\Sigma_1^{-1} + \Sigma_2^{-1})^{-1},
\qquad
\mu = \Sigma(\Sigma_1^{-1}\mu_1 + \Sigma_2^{-1}\mu_2)
$$

Dodatkowo:
- sumowany jest `observation_count`,
- typ jest uzupełniany (`unknown` -> znany, jeśli druga obserwacja ma znany typ).

Fuzja jest wydzielona do `fuseLandmarksInPlace()`.

### 1.3 Spójność grafu po merge (przepinanie krawędzi)
Po merge budowana jest mapa `removed_id -> kept_id`.

Następnie:
- rozwiązywane są łańcuchy mapowania (np. `A->B`, `B->C`),
- wszystkie `obs_edges_` są przepinane na docelowe ID landmarków,
- usuwane landmarki są kasowane z kontenera.

To eliminuje sytuacje, w których krawędzie obserwacji wskazują na nieistniejące wierzchołki.

### 1.4 Refaktoryzacja kodu merge
Logika merge została wydzielona z `optimizerLoop()` do metod:

- `mergeNearbyLandmarks()`
- `areLandmarksMergeCompatible()`
- `fuseLandmarksInPlace()`

Cel: czytelność i separacja odpowiedzialności.

---

## 2) Odometria odporna na skoki INS/RTK

### 2.1 Wygładzanie wejściowego strumienia odometrii
W `updateOdom()` dodano EMA dla `x`, `y`, `theta` (z poprawnym zawijaniem kąta):

$$
p_t = p_{t-1} + \alpha (p^{raw}_t - p_{t-1})
$$

gdzie dla yaw różnica liczona jest przez `normalizeAngle`.

Parametry:
- `odometry/smoothing_enabled`
- `odometry/smoothing_alpha`

Efekt: backend SLAM i publikacja pozycji operują na tym samym wygładzonym strumieniu odometrii.

### 2.2 Robust kernel dla krawędzi odometrii
Na `g2o::EdgeSE2` dodano Huber robust kernel.

Parametry:
- `odometry/use_robust_kernel`
- `odometry/robust_kernel_delta`

Efekt: pojedyncze odometryczne outliery mają ograniczony wpływ na rozwiązanie.

### 2.3 Adaptacyjna macierz informacji (gating przez info)
Wprowadzono adaptacyjne osłabianie informacji krawędzi odometrii na podstawie wielkości kroku.

Proces:
- liczony jest krok translacyjny i rotacyjny,
- liczony jest `ratio` względem progów,
- skala informacji maleje z grubsza jak $1/ratio^2$ dla `ratio>1`,
- dla dużych przekroczeń (`hard_gate_ratio`) stosowane jest dodatkowe tłumienie,
- skala jest ograniczona od dołu przez `min_info_scale`.

Parametry:
- `odometry/adaptive_info_enabled`
- `odometry/translation_gate`
- `odometry/rotation_gate`
- `odometry/hard_gate_ratio`
- `odometry/min_info_scale`

Logika została wydzielona do `computeOdometryInformation(dx, dy, dtheta)` i używana przy tworzeniu każdej `OdomEdge`.

---

## 3) Wygładzanie offsetu `map->odom` i użycie w całym pipeline

### 3.1 Wygładzanie aktualizacji offsetu
Offset obliczany po optymalizacji (`calculateOffset(raw_ins_pose, optimized_pose)`) jest wygładzany EMA, zamiast nadpisywania skokowego.

Parametry:
- `odometry/offset_smoothing_enabled`
- `odometry/offset_smoothing_alpha`

### 3.2 Spójne użycie offsetu i odometrii przy publikacji
Publikacja pose/TF korzysta z odometrii pobranej przez `lookupPose()` (czyli tej samej, którą backend używa do budowy grafu), a następnie z aktualnego `map_to_odom_offset_`.

Efekt:
- spójność między backendem SLAM a publikowanym `/slam/pose` i TF,
- mniejsze skoki na wyjściu.

---

## 4) Aktualny zestaw parametrów dodanych/rozszerzonych

W `config/graphslam_params.yaml` rozszerzono sekcję `odometry` o:

- `smoothing_enabled`
- `smoothing_alpha`
- `use_robust_kernel`
- `robust_kernel_delta`
- `adaptive_info_enabled`
- `translation_gate`
- `rotation_gate`
- `hard_gate_ratio`
- `min_info_scale`
- `offset_smoothing_enabled`
- `offset_smoothing_alpha`

oraz aktywnie wykorzystywany jest `data_association/use_mahalanobis` i `data_association/mahalanobis_threshold` w logice merge landmarków.

---

## 5) Miejsca w kodzie objęte zmianami

- `include/GraphManager.h`
  - nowe pola konfiguracyjne i stanowe,
  - nowe metody pomocnicze dla merge i informacji odometrii.

- `src/GraphManager.cpp`
  - implementacja merge landmarków (kompatybilność + fuzja + rewiring),
  - EMA wejściowej odometrii,
  - robust kernel dla `EdgeSE2`,
  - adaptacyjna informacja odometrii,
  - wygładzanie `map_to_odom_offset_`.

- `src/GraphSLAM.cpp`
  - publikacja corrected pose/TF oparta na `lookupPose()` (spójna z backendem).

- `config/graphslam_params.yaml`
  - parametry strojenia nowych mechanizmów.
