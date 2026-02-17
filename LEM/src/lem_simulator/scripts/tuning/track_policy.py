#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
track_policy.py
===============

Jedna odpowiedzialność: WYBÓR TORÓW do oceny jednego theta.

To NIE ma nic wspólnego z Bayesem ani z uruchamianiem ROS.
To jest tylko "design eksperymentu": single / fixed list / random / curriculum.

Zwracam listę track_id, które bayes_tune ma odpalić przez lem_eval.
"""

from dataclasses import dataclass
from typing import Any, Dict, List
import random


def _clamp_int(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(x)))


@dataclass
class TrackPolicy:
    mode: str
    tmin: int
    tmax: int
    tracks_per_theta: int
    no_replacement: bool

    single_track_id: int
    fixed_list: List[int]

    # proste curriculum: start od 1 toru, potem rośnie do tracks_per_theta
    curriculum_warmup_evals: int
    curriculum_grow_every: int

    @staticmethod
    def from_cfg(cfg: Dict[str, Any]) -> "TrackPolicy":
        tracks_cfg = cfg.get("tracks", {})
        tmin = int(tracks_cfg.get("track_index_min", 1))
        tmax = int(tracks_cfg.get("track_index_max", 1))

        eval_cfg = cfg.get("eval", {})
        k = int(eval_cfg.get("tracks_per_theta", 1))

        pol = cfg.get("track_policy", {})
        mode = str(pol.get("mode", "random_range"))

        no_rep = bool(pol.get("random_no_replacement", False))

        single_id = int(pol.get("single_track_id", tmin))
        single_id = _clamp_int(single_id, tmin, tmax)

        fixed_list = list(pol.get("fixed_list", []))
        fixed_list = [int(x) for x in fixed_list if tmin <= int(x) <= tmax]
        if not fixed_list:
            fixed_list = [single_id]

        warm = int(pol.get("curriculum_warmup_evals", 10))
        grow_every = int(pol.get("curriculum_grow_every", 5))

        return TrackPolicy(
            mode=mode,
            tmin=tmin,
            tmax=tmax,
            tracks_per_theta=max(1, k),
            no_replacement=no_rep,
            single_track_id=single_id,
            fixed_list=fixed_list,
            curriculum_warmup_evals=max(0, warm),
            curriculum_grow_every=max(1, grow_every),
        )

    def pick_tracks(self, *, eval_index: int, tag: str, rng: random.Random) -> List[int]:
        """
        eval_index: 1..N (kolejny numer ewaluacji w całym procesie)
        tag: "init"/"bo_step"/...
        """
        mode = self.mode.lower().strip()

        if mode == "single":
            return [self.single_track_id]

        if mode == "fixed_list":
            # deterministycznie: bierz pierwsze K
            # jeśli chcesz losowo z listy, ustaw mode="fixed_list_random"
            return self.fixed_list[: self.tracks_per_theta]

        if mode == "fixed_list_random":
            k = min(self.tracks_per_theta, len(self.fixed_list))
            if self.no_replacement:
                return rng.sample(self.fixed_list, k=k)
            return [rng.choice(self.fixed_list) for _ in range(self.tracks_per_theta)]

        if mode == "curriculum":
            # warmup: 1 tor (single), potem co curriculum_grow_every ewaluacji zwiększaj K aż do tracks_per_theta
            if eval_index <= self.curriculum_warmup_evals:
                return [self.single_track_id]

            steps_after = eval_index - self.curriculum_warmup_evals
            k = 1 + (steps_after // self.curriculum_grow_every)
            k = max(1, min(self.tracks_per_theta, k))

            # losuj k torów z zakresu
            return self._sample_range(k, rng)

        # default: random_range
        return self._sample_range(self.tracks_per_theta, rng)

    def _sample_range(self, k: int, rng: random.Random) -> List[int]:
        k = max(1, int(k))
        n = (self.tmax - self.tmin + 1)

        if self.no_replacement and k <= n:
            return rng.sample(list(range(self.tmin, self.tmax + 1)), k=k)

        return [rng.randint(self.tmin, self.tmax) for _ in range(k)]