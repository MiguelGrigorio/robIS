#!/usr/bin/env python3
"""Summarize localization-evaluator CSV files into article-ready metrics."""

import argparse
import csv
import json
import math
from pathlib import Path
import re

import numpy as np


def summarize(rows):
    """Group errors by method/class, retaining trial-level independence."""
    grouped = {}
    trials = {}
    footprint_grouped = {}
    for row in rows:
        key = (row["range_estimator"], row["label"])
        error = float(row["error_m"])
        grouped.setdefault(key, []).append(error)
        trial_key = key + (row.get("trial_id") or "legacy",)
        trials.setdefault(trial_key, []).append(error)
        footprint_error = row.get("footprint_error_m")
        if footprint_error not in (None, ""):
            footprint_grouped.setdefault(key, []).append(float(footprint_error))
    report = []
    for (estimator, label), errors in sorted(grouped.items()):
        values = np.asarray(errors, dtype=float)
        trial_medians = np.asarray([
            np.median(trial_errors)
            for (trial_estimator, trial_label, _), trial_errors in trials.items()
            if trial_estimator == estimator and trial_label == label
        ])
        trial_values = {
            trial_id: float(np.median(trial_errors))
            for (trial_estimator, trial_label, trial_id), trial_errors
            in sorted(trials.items())
            if trial_estimator == estimator and trial_label == label
        }
        item = {
            "range_estimator": estimator,
            "label": label,
            "samples": int(values.size),
            "trials": int(trial_medians.size),
            "mean_error_m": float(values.mean()),
            "median_error_m": float(np.median(values)),
            "p95_error_m": float(np.percentile(values, 95)),
            "rmse_m": float(np.sqrt(np.mean(values ** 2))),
            "within_0_5m": float(np.mean(values <= 0.5)),
            "trial_median_mean_m": float(trial_medians.mean()),
            "trial_median_std_m": float(trial_medians.std(ddof=1))
            if trial_medians.size > 1 else 0.0,
            "trial_median_error_m": trial_values,
        }
        footprint_errors = np.asarray(
            footprint_grouped.get((estimator, label), []), dtype=float
        )
        item.update({
            "footprint_samples": int(footprint_errors.size),
            "median_footprint_error_m": (
                float(np.median(footprint_errors))
                if footprint_errors.size else None
            ),
            "p95_footprint_error_m": (
                float(np.percentile(footprint_errors, 95))
                if footprint_errors.size else None
            ),
            "within_footprint_0_1m": (
                float(np.mean(footprint_errors <= 0.1))
                if footprint_errors.size else None
            ),
        })
        report.append(item)
    return report


def summarize_diagnostics(rows):
    """Report association yield and categorized failures per method/class."""
    grouped = {}
    for row in rows:
        key = (row["range_estimator"], row["label"])
        grouped.setdefault(key, []).append(row)
    report = []
    for (estimator, label), items in sorted(grouped.items()):
        localized = [item for item in items if item["localized"].lower() == "true"]
        range_errors = [
            float(item["range_error_m"])
            for item in localized
            if item.get("range_error_m") not in (None, "")
        ]
        bearing_errors = [
            float(item["bearing_error_rad"])
            for item in localized
            if item.get("bearing_error_rad") not in (None, "")
        ]
        reasons = {}
        for item in items:
            reason = item.get("failure_reason") or "localized"
            reasons[reason] = reasons.get(reason, 0) + 1
        per_trial = {}
        for item in items:
            trial = item.get("trial_id") or "legacy"
            metrics = per_trial.setdefault(trial, {"attempts": 0, "localized": 0})
            metrics["attempts"] += 1
            metrics["localized"] += item["localized"].lower() == "true"
        for metrics in per_trial.values():
            metrics["localization_yield"] = (
                metrics["localized"] / max(metrics["attempts"], 1)
            )
        trial_range_errors = {}
        trial_bearing_errors = {}
        for item in localized:
            if item.get("range_error_m") in (None, ""):
                continue
            trial = item.get("trial_id") or "legacy"
            trial_range_errors.setdefault(trial, []).append(
                float(item["range_error_m"])
            )
            if item.get("bearing_error_rad") not in (None, ""):
                trial_bearing_errors.setdefault(trial, []).append(
                    float(item["bearing_error_rad"])
                )
        trial_range_medians = {
            trial: float(np.median(values))
            for trial, values in sorted(trial_range_errors.items())
        }
        trial_bearing_medians = {
            trial: float(np.degrees(np.median(values)))
            for trial, values in sorted(trial_bearing_errors.items())
        }
        report.append({
            "range_estimator": estimator,
            "label": label,
            "attempts": len(items),
            "localized": len(localized),
            "localization_yield": len(localized) / max(len(items), 1),
            "failure_reasons": reasons,
            "range_error_samples": len(range_errors),
            "median_range_error_m": (
                float(np.median(range_errors)) if range_errors else None
            ),
            "p95_range_error_m": (
                float(np.percentile(range_errors, 95)) if range_errors else None
            ),
            "bearing_error_samples": len(bearing_errors),
            "median_bearing_error_deg": (
                float(np.degrees(np.median(bearing_errors)))
                if bearing_errors else None
            ),
            "p95_bearing_error_deg": (
                float(np.degrees(np.percentile(bearing_errors, 95)))
                if bearing_errors else None
            ),
            "trial_median_range_error_m": trial_range_medians,
            "trial_median_bearing_error_deg": trial_bearing_medians,
            "trials": per_trial,
        })
    return report


def paired_range_comparisons(
        rows, reference_estimator="bbox_cluster", bootstrap_resamples=10000,
        bootstrap_seed=20260822):
    """Compare per-trial medians using sign test and paired bootstrap CIs."""
    grouped = {}
    labels = set()
    estimators = set()
    for row in rows:
        value = row.get("range_error_m")
        if row.get("localized", "").lower() != "true" or value in (None, ""):
            continue
        estimator, label = row["range_estimator"], row["label"]
        trial = row.get("trial_id") or "legacy"
        suffix = f"_{estimator}"
        base_trial = trial[:-len(suffix)] if trial.endswith(suffix) else trial
        grouped.setdefault((estimator, label, base_trial), []).append(float(value))
        labels.add(label)
        estimators.add(estimator)
    medians = {key: float(np.median(values)) for key, values in grouped.items()}
    comparisons = []
    for label in sorted(labels):
        for estimator in sorted(estimators - {reference_estimator}):
            pairs = []
            for (method, trial_label, trial), value in medians.items():
                if method != reference_estimator or trial_label != label:
                    continue
                other = medians.get((estimator, label, trial))
                if other is not None:
                    pairs.append((trial, value, other))
            wins = sum(reference < other for _, reference, other in pairs)
            losses = sum(reference > other for _, reference, other in pairs)
            non_ties = wins + losses
            tail = min(wins, losses)
            p_value = (
                min(1.0, 2.0 * sum(
                    math.comb(non_ties, index) for index in range(tail + 1)
                ) / (2 ** non_ties))
                if non_ties else 1.0
            )
            differences = np.asarray([
                reference - other for _, reference, other in pairs
            ], dtype=float)
            mean_ci = None
            median_ci = None
            if differences.size:
                rng = np.random.default_rng(bootstrap_seed)
                samples = rng.choice(
                    differences,
                    size=(int(bootstrap_resamples), differences.size),
                    replace=True,
                )
                mean_ci = [
                    float(value)
                    for value in np.percentile(samples.mean(axis=1), [2.5, 97.5])
                ]
                median_ci = [
                    float(value)
                    for value in np.percentile(np.median(samples, axis=1), [2.5, 97.5])
                ]
            grouped_differences = {}
            for trial, reference, other in pairs:
                match = re.match(r"^(.*)_r\d+$", trial)
                group = match.group(1) if match else trial
                grouped_differences.setdefault(group, []).append(reference - other)
            group_means = {
                group: float(np.mean(values))
                for group, values in sorted(grouped_differences.items())
            }
            group_values = np.asarray(list(group_means.values()), dtype=float)
            group_wins = int(np.sum(group_values < 0.0))
            group_losses = int(np.sum(group_values > 0.0))
            group_non_ties = group_wins + group_losses
            group_tail = min(group_wins, group_losses)
            group_p_value = (
                min(1.0, 2.0 * sum(
                    math.comb(group_non_ties, index)
                    for index in range(group_tail + 1)
                ) / (2 ** group_non_ties))
                if group_non_ties else 1.0
            )
            group_mean_ci = None
            if group_values.size:
                group_rng = np.random.default_rng(bootstrap_seed)
                group_samples = group_rng.choice(
                    group_values,
                    size=(int(bootstrap_resamples), group_values.size),
                    replace=True,
                )
                group_mean_ci = [
                    float(value)
                    for value in np.percentile(
                        group_samples.mean(axis=1), [2.5, 97.5]
                    )
                ]
            comparisons.append({
                "label": label,
                "reference_estimator": reference_estimator,
                "other_estimator": estimator,
                "paired_trials": len(pairs),
                "reference_wins": wins,
                "other_wins": losses,
                "ties": len(pairs) - non_ties,
                "mean_paired_difference_m": (
                    float(differences.mean()) if differences.size else None
                ),
                "mean_paired_difference_ci95_m": mean_ci,
                "median_paired_difference_m": (
                    float(np.median(differences)) if differences.size else None
                ),
                "median_paired_difference_ci95_m": median_ci,
                "difference_definition": "reference_minus_other",
                "bootstrap_resamples": int(bootstrap_resamples),
                "bootstrap_seed": int(bootstrap_seed),
                "exact_two_sided_sign_p": p_value,
                "trial_grouping_rule": "remove_trailing_rNN",
                "trial_groups": int(group_values.size),
                "group_mean_paired_difference_m": (
                    float(group_values.mean()) if group_values.size else None
                ),
                "group_mean_paired_difference_ci95_m": group_mean_ci,
                "group_paired_difference_m": group_means,
                "group_reference_wins": group_wins,
                "group_other_wins": group_losses,
                "group_ties": int(group_values.size) - group_non_ties,
                "group_exact_two_sided_sign_p": group_p_value,
            })
    return comparisons


def paired_sample_comparisons(rows, metric):
    """Pair trial medians for an evaluator metric such as center error."""
    normalized = []
    for row in rows:
        value = row.get(metric)
        if value in (None, ""):
            continue
        normalized.append({
            "range_estimator": row["range_estimator"],
            "label": row["label"],
            "trial_id": row.get("trial_id", ""),
            "localized": "true",
            "range_error_m": value,
        })
    comparisons = paired_range_comparisons(normalized)
    for comparison in comparisons:
        comparison["metric"] = metric
    return comparisons


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--diagnostics", type=Path, nargs="+",
        help="One or more diagnostics CSVs from compatible campaigns",
    )
    parsed = parser.parse_args(args=args)
    rows = []
    for filename in parsed.inputs:
        with filename.open("r", encoding="utf-8", newline="") as stream:
            rows.extend(csv.DictReader(stream))
    report = {"files": [str(item) for item in parsed.inputs], "groups": summarize(rows)}
    report["paired_center_error_comparisons"] = paired_sample_comparisons(
        rows, "error_m"
    )
    report["paired_footprint_error_comparisons"] = paired_sample_comparisons(
        rows, "footprint_error_m"
    )
    if parsed.diagnostics:
        diagnostic_rows = []
        for filename in parsed.diagnostics:
            with filename.open(
                    "r", encoding="utf-8", newline="") as stream:
                diagnostic_rows.extend(csv.DictReader(stream))
        report["diagnostics_files"] = [
            str(filename) for filename in parsed.diagnostics
        ]
        if len(parsed.diagnostics) == 1:
            report["diagnostics_file"] = str(parsed.diagnostics[0])
        report["diagnostics"] = summarize_diagnostics(diagnostic_rows)
        report["paired_range_comparisons"] = paired_range_comparisons(
            diagnostic_rows
        )
    parsed.output.parent.mkdir(parents=True, exist_ok=True)
    parsed.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
