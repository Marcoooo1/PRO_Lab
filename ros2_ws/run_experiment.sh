#!/usr/bin/env bash
# =============================================================================
# run_experiment.sh  –  Experiment starten + Ergebnisse automatisch speichern
#
# Aufruf:
#   bash run_experiment.sh <tag> [config] [pattern] [duration] [v] [w]
#
# Beispiele:
#   bash run_experiment.sh wrong_init_circle
#   bash run_experiment.sh correct_init_circle filters_correct_init.yaml
#   bash run_experiment.sh wrong_init_eight   filters_wrong_init.yaml eight 90.0
#   bash run_experiment.sh wrong_init_fast    filters_wrong_init.yaml circle 60.0 0.5 0.5
# =============================================================================
set -e

TAG="${1:-run}"
CONFIG="${2:-filters_wrong_init.yaml}"
PATTERN="${3:-circle}"
DURATION="${4:-60.0}"
LINEAR_V="${5:-0.2}"
ANGULAR_V="${6:-0.3}"

WS=~/PROL/ros2_ws
RESULTS=~/PROL/results/${TAG}
CSV=${RESULTS}/data.csv
PLOTS=${RESULTS}
EVAL=${WS}/install/prob_robotics_eval/lib/prob_robotics_eval/evaluate.py

# Setup
source ${WS}/install/setup.bash
mkdir -p "${RESULTS}"

echo "============================================"
echo " Experiment: ${TAG}"
echo " Config:     ${CONFIG}"
echo " Pattern:    ${PATTERN}"
echo " Duration:   ${DURATION}s"
echo " Speed:      v=${LINEAR_V}  w=${ANGULAR_V}"
echo " Ergebnisse: ${RESULTS}/"
echo "============================================"

# Filter + AutoDriver + Logger starten
echo ""
echo "HINWEIS: Druecke Ctrl+C sobald der AutoDriver 'Trajektorie beendet' meldet!"
echo ""
ros2 launch prob_robotics_bringup all_filters.launch.py \
    config_file:=${CONFIG} \
    pattern:=${PATTERN} \
    duration:=${DURATION} \
    csv_path:=${CSV} \
    linear_v:=${LINEAR_V} \
    angular_v:=${ANGULAR_V}

# Nach Ctrl+C oder AutoDriver-Ende: auswerten
echo ""
echo "==> Auswertung laeuft..."
python3 "${EVAL}" "${CSV}" --tag "${TAG}" --out "${PLOTS}"

echo ""
echo "============================================"
echo " Fertig! Ergebnisse in:"
echo "   ${RESULTS}/"
echo " Windows: \\\\wsl.localhost\\Ubuntu${RESULTS/#\~//home/marco}"
echo "============================================"
