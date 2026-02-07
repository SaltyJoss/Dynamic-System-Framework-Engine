#!/bin/bash

# PR Merge Readiness Checker
# This script helps you evaluate if your PR is ready to merge
# despite automated review comments

echo "═══════════════════════════════════════════════════════════"
echo "  PR Merge Readiness Checker"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Function to ask yes/no questions
ask_question() {
    local question=$1
    local answer
    echo -n "$question (y/n): "
    read answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        return 0
    else
        return 1
    fi
}

# Critical checks
echo "CRITICAL CHECKS (must all be YES to merge):"
echo "-----------------------------------------------------------"

all_critical_pass=true

if ask_question "Does the build pass?"; then
    echo "  ✓ Build passes"
else
    echo "  ✗ Build fails - DO NOT MERGE"
    all_critical_pass=false
fi

if ask_question "Do all tests pass (or N/A)?"; then
    echo "  ✓ Tests pass"
else
    echo "  ✗ Tests fail - DO NOT MERGE"
    all_critical_pass=false
fi

if ask_question "Have you reviewed your own code?"; then
    echo "  ✓ Self-reviewed"
else
    echo "  ✗ Not reviewed - REVIEW BEFORE MERGE"
    all_critical_pass=false
fi

if ask_question "Does the code achieve its stated purpose?"; then
    echo "  ✓ Purpose achieved"
else
    echo "  ✗ Purpose not achieved - DO NOT MERGE"
    all_critical_pass=false
fi

if ask_question "Are there any actual bugs in the code?"; then
    echo "  ✗ Bugs present - FIX BEFORE MERGE"
    all_critical_pass=false
else
    echo "  ✓ No bugs"
fi

if ask_question "Are there any security vulnerabilities?"; then
    echo "  ✗ Security issues - FIX BEFORE MERGE"
    all_critical_pass=false
else
    echo "  ✓ No security issues"
fi

echo ""
echo "NON-BLOCKING CHECKS:"
echo "-----------------------------------------------------------"

if ask_question "Are there automated review comments?"; then
    echo ""
    echo "  Review comment types (these DON'T block merging):"
    echo "    • Trivial typos (Github→GitHub, etc.) - can dismiss"
    echo "    • Style preferences - can dismiss"
    echo "    • Suggestions without context - can dismiss"
    echo "    • Minor code improvements - can defer to later"
    echo ""
    
    if ask_question "  Have you evaluated all automated comments?"; then
        echo "    ✓ Comments evaluated"
        if ask_question "  Did you address legitimate issues only?"; then
            echo "    ✓ Legitimate issues addressed"
        fi
    fi
fi

echo ""
echo "═══════════════════════════════════════════════════════════"

if [ "$all_critical_pass" = true ]; then
    echo "VERDICT: ✅ READY TO MERGE"
    echo ""
    echo "Your PR meets all critical requirements."
    echo "Unresolved automated comments for typos, style, etc."
    echo "do NOT need to block your merge."
else
    echo "VERDICT: ❌ NOT READY TO MERGE"
    echo ""
    echo "Please address the critical issues above before merging."
fi

echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Remember: YOU decide when your code is ready, not an"
echo "automated reviewer. Trust your professional judgment."
echo ""
