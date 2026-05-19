complete -c mull-reporter-18 -l reporters -d 'Output reporters to use, more than one can be used at the same time' -r -f -a "IDE\t'IDE-friendly output with file:line:column format'
SQLite\t'SQLite database for offline analysis'
GitHubAnnotations\t'GitHub Actions annotation format'
Patches\t'Generate patch files for each mutation'
Elements\t'Mutation Testing Elements JSON/HTML report'
Sarif\t'SARIF 2.1.0 report (Static Analysis Results Interchange Format)'"
complete -c mull-reporter-18 -l report-dir -d 'Directory for report output files' -r
complete -c mull-reporter-18 -l report-name -d 'Filename for the report (only for supported reporters)' -r
complete -c mull-reporter-18 -l report-patch-base -d 'Base directory for patch file paths' -r
complete -c mull-reporter-18 -l mutation-score-threshold -d 'Minimum mutation score (0-100) required for success' -r
complete -c mull-reporter-18 -l sqlite-busy-timeout -d 'SQLite reporter busy timeout in milliseconds. When the database is locked by another writer, mull will retry for this long before giving up with "database is locked"' -r
complete -c mull-reporter-18 -l ide-reporter-show-killed -d 'Show killed mutations in IDE reporter output'
complete -c mull-reporter-18 -l debug -d 'Enable debug mode with additional diagnostic output'
complete -c mull-reporter-18 -l strict -d 'Treat warnings as fatal errors'
complete -c mull-reporter-18 -l allow-surviving -d 'Do not treat surviving mutants as an error'
complete -c mull-reporter-18 -l no-test-output -d 'Does not capture output from test runs'
complete -c mull-reporter-18 -l no-mutant-output -d 'Does not capture output from mutant runs'
complete -c mull-reporter-18 -l no-output -d 'Combines --no-test-output and --no-mutant-output'
complete -c mull-reporter-18 -s h -l help -d 'Print help (see more with \'--help\')'
complete -c mull-reporter-18 -s V -l version -d 'Print version'
