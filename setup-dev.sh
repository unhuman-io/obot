#!/bin/bash
# Sets up the git clones of a workspace for development. Per clone, and per
# submodule of the same organization (a relative url in .gitmodules):
#   - the pre-commit hook, where the repo has a .pre-commit-config.yaml: it
#     formats the C and C++ files of a commit with clang-format, at the
#     version CI checks, and stops the commit if that changed any
#   - git blame passes over the commits of .git-blame-ignore-revs, the
#     formatting passes, where the repo has that file
# Installs pre-commit with pipx if it is missing. These are settings of a
# clone, not of the repo: run it once after cloning, and again after adding a
# clone or a submodule. Running it again changes nothing.
#
#   ./setup-dev.sh [-n] [directory...]
#
# directory: a clone, or a directory of clones. Default: the directory this
#            repo is in, the workspace of the repo manifest.
# -n:        only print what it would do

set -euo pipefail

dry_run=
if [ "${1:-}" = -n ]; then dry_run=1; shift; fi
[ $# -gt 0 ] || set -- "$(dirname "$(dirname "$(readlink -f "$0")")")"

run() {
    if [ -n "$dry_run" ]; then echo "would run: $*"; else "$@"; fi
}
indent() {
    sed 's/^/    /'
}

# Is the file in the checkout, or on a main branch of a remote? A clone on an
# older branch is set up too: the file arrives with its next rebase.
in_repo() {     # clone, file
    [ -e "$1/$2" ] && return 0
    local ref
    for ref in $(git -C "$1" for-each-ref --format='%(refname)' 'refs/remotes/*/HEAD' 'refs/remotes/*/develop' 'refs/remotes/*/main'); do
        git -C "$1" cat-file -e "$ref:$2" 2> /dev/null && return 0
    done
    return 1
}

# The clones: each directory given, or the clones in it, and their submodules.
# Not the third party ones, which have hooks and settings of their own.
repos=()
add_repo() {
    repos+=("$1")
    local sub
    while IFS= read -r sub; do
        add_repo "$sub"
    done < <(git -C "$1" submodule foreach --quiet \
        'case $(git config -f "$toplevel/.gitmodules" "submodule.$name.url") in ../*) echo "$toplevel/$sm_path" ;; esac')
}
for dir in "$@"; do
    dir=$(readlink -f "$dir")
    if [ -e "$dir/.git" ]; then
        add_repo "$dir"
    else
        for child in "$dir"/*/; do
            [ -e "$child.git" ] && add_repo "${child%/}"
        done
    fi
done
[ ${#repos[@]} -gt 0 ] || { echo "no git clone in: $*"; exit 1; }

if ! command -v pre-commit > /dev/null; then
    command -v pipx > /dev/null || { echo "pre-commit is not installed, and neither is pipx to install it with: sudo apt install pipx"; exit 1; }
    run pipx install pre-commit | indent
    PATH=$PATH:$(pipx environment --value PIPX_BIN_DIR)
fi

# git blame stops on an ignore file that is not there, as on a branch from
# before the file. From git 2.52 the setting can say that the file is optional.
optional=
if printf '2.52\n%s\n' "$(git version | cut -d' ' -f3)" | sort -V -C; then optional=":(optional)"; fi

failed=
for repo in "${repos[@]}"; do
    echo "$repo"
    if in_repo "$repo" .pre-commit-config.yaml; then
        # with --allow-missing-config a branch from before the file commits as it did
        (cd "$repo" && run pre-commit install --allow-missing-config | indent) || failed=1
    else
        echo "    no .pre-commit-config.yaml: no hook"
    fi
    if ! in_repo "$repo" .git-blame-ignore-revs; then
        echo "    no .git-blame-ignore-revs: git blame as it is"
    elif [ -n "$optional" ] || [ -e "$repo/.git-blame-ignore-revs" ]; then
        run git -C "$repo" config blame.ignoreRevsFile "$optional.git-blame-ignore-revs" | indent
        [ -n "$dry_run" ] || echo "    git blame passes over the commits of .git-blame-ignore-revs"
    else
        echo "    .git-blame-ignore-revs is not on this branch, and git is older than 2.52: run this again once it is"
    fi
done
[ -z "$failed" ] || { echo "not all of it worked: see above"; exit 1; }
