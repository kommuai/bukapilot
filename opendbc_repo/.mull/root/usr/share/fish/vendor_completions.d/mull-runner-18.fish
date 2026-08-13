complete -c mull-runner-18 -l test-program -d 'Path to a test program (if different from input executable)' -r
complete -c mull-runner-18 -l workers -d 'Number of parallel workers for mutation search' -r
complete -c mull-runner-18 -l timeout -d 'Timeout per test run in milliseconds' -r
complete -c mull-runner-18 -l ld-search-path -d 'Library search path' -r
complete -c mull-runner-18 -l coverage-info -d 'Path to the coverage info file (LLVM profdata)' -r
complete -c mull-runner-18 -l minimum-timeout -d 'Minimum timeout per mutant run in milliseconds. The actual timeout is max(baseline*10, minimum-timeout)' -r
complete -c mull-runner-18 -l reporters -d 'Output reporters to use, more than one can be used at the same time' -r -f -a "IDE\t'IDE-friendly output with file:line:column format'
SQLite\t'SQLite database for offline analysis'
GitHubAnnotations\t'GitHub Actions annotation format'
Patches\t'Generate patch files for each mutation'
Elements\t'Mutation Testing Elements JSON/HTML report'
Sarif\t'SARIF 2.1.0 report (Static Analysis Results Interchange Format)'"
complete -c mull-runner-18 -l report-dir -d 'Directory for report output files' -r
complete -c mull-runner-18 -l report-name -d 'Filename for the report (only for supported reporters)' -r
complete -c mull-runner-18 -l report-patch-base -d 'Base directory for patch file paths' -r
complete -c mull-runner-18 -l mutation-score-threshold -d 'Minimum mutation score (0-100) required for success' -r
complete -c mull-runner-18 -l sqlite-busy-timeout -d 'SQLite reporter busy timeout in milliseconds. When the database is locked by another writer, mull will retry for this long before giving up with "database is locked"' -r
complete -c mull-runner-18 -l include-not-covered -d 'Include mutants on lines not covered by tests'
complete -c mull-runner-18 -l dry-run -d 'Skip mutant execution, only discover and report mutants'
complete -c mull-runner-18 -l debug-coverage -d 'Print coverage ranges'
complete -c mull-runner-18 -l ide-reporter-show-killed -d 'Show killed mutations in IDE reporter output'
complete -c mull-runner-18 -l debug -d 'Enable debug mode with additional diagnostic output'
complete -c mull-runner-18 -l strict -d 'Treat warnings as fatal errors'
complete -c mull-runner-18 -l allow-surviving -d 'Do not treat surviving mutants as an error'
complete -c mull-runner-18 -l no-test-output -d 'Does not capture output from test runs'
complete -c mull-runner-18 -l no-mutant-output -d 'Does not capture output from mutant runs'
complete -c mull-runner-18 -l no-output -d 'Combines --no-test-output and --no-mutant-output'
complete -c mull-runner-18 -s h -l help -d 'Print help (see more with \'--help\')'
complete -c mull-runner-18 -s V -l version -d 'Print version'
