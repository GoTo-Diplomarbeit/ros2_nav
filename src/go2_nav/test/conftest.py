def pytest_terminal_summary(terminalreporter, exitstatus, config):
    passed = len(terminalreporter.stats.get("passed", []))
    failed = len(terminalreporter.stats.get("failed", []))
    skipped = len(terminalreporter.stats.get("skipped", []))
    total = passed + failed + skipped

    lines = []
    lines.append("## Test Results")
    lines.append("")

    if failed == 0:
        lines.append(f"All {total} tests passed.")
    else:
        lines.append(f"{failed} of {total} tests failed.")

    lines.append("")
    lines.append("| Passed | Failed | Skipped |")
    lines.append("|--------|--------|---------|")
    lines.append(f"| {passed} | {failed} | {skipped} |")
    lines.append("")

    for report in terminalreporter.stats.get("passed", []):
        lines.append(f"- PASS: `{report.nodeid}`")
    for report in terminalreporter.stats.get("failed", []):
        lines.append(f"- FAIL: `{report.nodeid}`")

    summary = "\n".join(lines)

    with open("test_summary.md", "w") as f:
        f.write(summary)

    print("\n" + "=" * 50)
    print("Markdown summary written to test_summary.md")
    print("=" * 50)
