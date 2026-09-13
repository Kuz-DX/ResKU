# This file is used as Bash's --rcfile inside each tmux pane.
# Load the user's normal interactive configuration first.
if [[ -r "${HOME}/.bashrc" ]]; then
    # shellcheck disable=SC1090
    source "${HOME}/.bashrc"
fi

# A .bashrc is allowed to change HISTFILE. Restore the pane-specific file so
# concurrent panes do not overwrite or reorder one another's command history.
if [[ -n "${DOLBOTZ_HISTORY_FILE:-}" ]]; then
    HISTFILE="${DOLBOTZ_HISTORY_FILE}"
    export HISTFILE
fi

shopt -s histappend
HISTSIZE="${HISTSIZE:-5000}"
HISTFILESIZE="${HISTFILESIZE:-10000}"

# Persist each executed line immediately. The in-memory Bash history is what
# makes Up restore the command directly after Ctrl+C; HISTFILE also preserves
# it when this launcher is opened again.
__dolbotz_launcher_history_sync() {
    builtin history -a
}

if declare -p PROMPT_COMMAND 2>/dev/null | grep -q '^declare -a'; then
    PROMPT_COMMAND=(__dolbotz_launcher_history_sync "${PROMPT_COMMAND[@]}")
else
    PROMPT_COMMAND="__dolbotz_launcher_history_sync${PROMPT_COMMAND:+;${PROMPT_COMMAND}}"
fi
