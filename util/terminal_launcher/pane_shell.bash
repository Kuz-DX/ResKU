# This file is used as Bash's --rcfile inside each tmux pane.
# Load the user's normal interactive configuration first.
if [[ -r "${HOME}/.bashrc" ]]; then
    # shellcheck disable=SC1090
    source "${HOME}/.bashrc"
fi

# A .bashrc is allowed to change HISTFILE. Restore the pane-specific file so
# concurrent panes do not overwrite or reorder one another's command history.
if [[ -n "${RESKU_HISTORY_FILE:-}" ]]; then
    HISTFILE="${RESKU_HISTORY_FILE}"
    export HISTFILE
fi

shopt -s histappend
HISTSIZE="${HISTSIZE:-5000}"
HISTFILESIZE="${HISTFILESIZE:-10000}"

# Use an explicit tmux 256-colour palette entry instead of an RGB hex value.
# Some Tilix/tmux combinations do not advertise the RGB terminal feature and
# quantize a dark hex colour to black. colour28 is a visible medium-dark green
# while keeping normal light foreground text readable.
__resku_launcher_set_running_style() {
    [[ -n "${TMUX:-}" && -n "${TMUX_PANE:-}" ]] || return 0
    printf '\033]11;#008700\007'
    command tmux set-option -p -q -t "${TMUX_PANE}" \
        window-style 'bg=colour28' >/dev/null 2>&1 || true
    command tmux set-option -p -q -t "${TMUX_PANE}" \
        window-active-style 'bg=colour28' >/dev/null 2>&1 || true
}

__resku_launcher_set_idle_style() {
    [[ -n "${TMUX:-}" && -n "${TMUX_PANE:-}" ]] || return 0
    printf '\033]11;#5f0000\007'
    # colour52 is a dark red that remains distinguishable from the running
    # green while preserving the readability of normal light terminal text.
    command tmux set-option -p -q -t "${TMUX_PANE}" \
        window-style 'bg=colour52' >/dev/null 2>&1 || true
    command tmux set-option -p -q -t "${TMUX_PANE}" \
        window-active-style 'bg=colour52' >/dev/null 2>&1 || true
}

# PS0 is expanded after Enter is pressed and immediately before Bash executes
# the command. Nothing is printed because this function only updates tmux.
__resku_launcher_command_started() {
    __resku_launcher_set_running_style
}

# Persist each executed line immediately. The in-memory Bash history is what
# makes Up restore the command directly after Ctrl+C; HISTFILE also preserves
# it when this launcher is opened again. Reaching PROMPT_COMMAND also means the
# foreground command finished (normally or via Ctrl+C), so switch to the idle
# red pane style here.
__resku_launcher_prompt_ready() {
    builtin history -a
    __resku_launcher_set_idle_style
}

# Preserve a user-defined PS0 while prepending the launcher's invisible status
# update. The literal command substitution is intentionally evaluated later,
# each time Bash is about to execute an entered command.
PS0='$(__resku_launcher_command_started)'"${PS0-}"

if declare -p PROMPT_COMMAND 2>/dev/null | grep -q '^declare -a'; then
    PROMPT_COMMAND=(__resku_launcher_prompt_ready "${PROMPT_COMMAND[@]}")
else
    PROMPT_COMMAND="__resku_launcher_prompt_ready${PROMPT_COMMAND:+;${PROMPT_COMMAND}}"
fi
