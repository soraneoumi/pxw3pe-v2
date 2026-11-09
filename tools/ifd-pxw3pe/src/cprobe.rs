//! Test support: evaluates C expressions against real headers with the system C compiler, so
//! that the hand-written Rust mirrors are checked against the declarations they copy instead of
//! against a second hand-written copy.

use std::fmt::Write as _;
use std::fs;
use std::path::PathBuf;
use std::process::Command;

/// Compiles and runs a program that prints each expression of `checks` (cast to `long long`,
/// which holds every value checked) and asserts that it equals the paired value.
pub(crate) fn assert_values(
    tag: &str,
    headers: &[&str],
    include_dirs: &[&str],
    checks: &[(&str, i128)],
) {
    let mut src = String::new();
    for header in headers {
        writeln!(src, "#include <{header}>").unwrap();
    }
    src.push_str("#include <stdio.h>\nint main(void)\n{\n");
    for (expr, _) in checks {
        writeln!(src, "\tprintf(\"%lld\\n\", (long long)({expr}));").unwrap();
    }
    src.push_str("\treturn 0;\n}\n");

    let dir: PathBuf =
        std::env::temp_dir().join(format!("ifd-pxw3pe-{tag}-{}", std::process::id()));
    fs::create_dir_all(&dir).unwrap();
    let source = dir.join("probe.c");
    let binary = dir.join("probe");
    fs::write(&source, &src).unwrap();

    let mut cc = Command::new("cc");
    for include in include_dirs {
        cc.arg("-I").arg(include);
    }
    let compiled = cc
        .arg("-o")
        .arg(&binary)
        .arg(&source)
        .output()
        .unwrap_or_else(|e| {
            panic!("cannot run the C compiler `cc`: {e}");
        });
    assert!(
        compiled.status.success(),
        "compiling the {tag} probe failed (are the headers installed?):\n{}",
        String::from_utf8_lossy(&compiled.stderr)
    );
    let run = Command::new(&binary).output().unwrap();
    fs::remove_dir_all(&dir).unwrap();
    assert!(run.status.success());

    let stdout = String::from_utf8(run.stdout).unwrap();
    let values: Vec<i128> = stdout.lines().map(|line| line.parse().unwrap()).collect();
    assert_eq!(values.len(), checks.len());
    for ((expr, expected), actual) in checks.iter().zip(values) {
        assert_eq!(
            actual, *expected,
            "{expr}: C says {actual:#x}, Rust says {expected:#x}"
        );
    }
}
