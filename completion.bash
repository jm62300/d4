_d4_completions()
{
    local cur opts exec_cmd
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"

    exec_cmd="${COMP_WORDS[0]}"

    local prev="${COMP_WORDS[COMP_CWORD-1]}"
    if [[ ${prev} == "-i" || ${prev} == "--inputName" ]] ; then
        COMPREPLY=( $(compgen -f -X "!*.cnf" -- "${cur}") $(compgen -d -- "${cur}") )
        return 0
    fi

    if [[ ${cur} == -* ]] ; then
        opts=$($exec_cmd --dump-options 2>/dev/null)
        
        if [[ -n "$opts" ]]; then
            COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
        fi
        return 0
    fi
}

# Associe la fonction d'autocomplétion à tous les exécutables D4
for bin in counter compiler maxT qbfCounter server competition demoSave; do
    complete -o default -F _d4_completions "$bin"
    complete -o default -F _d4_completions "./$bin"
    complete -o default -F _d4_completions "build/$bin"
    complete -o default -F _d4_completions "./build/$bin"
done
