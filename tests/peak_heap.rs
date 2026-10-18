//! What the gates' heap counter reads.
//!
//! The counter is the process's, so a case that asserts a rise to the byte
//! reads short by whatever any other thread frees inside its window, and the
//! test harness's own threads free on every result they report. These cases
//! therefore run in a binary of their own with no harness: one thread, one
//! case after another, and nothing else allocating while a window is open.
//!
//! The runner answers `--list` in the harness's terse form and takes an exact
//! name, so `cargo test` and nextest both drive it.

mod bitwise;

use bitwise::baseline::{anchor_peak, heap_turn, peak_bytes, try_heap_turn};

const CASES: &[(&str, fn())] = &[
    (
        "the_counting_allocator_sees_the_heap",
        the_counting_allocator_sees_the_heap,
    ),
    (
        "an_anchored_peak_measures_the_rise_not_the_heap_already_held",
        an_anchored_peak_measures_the_rise_not_the_heap_already_held,
    ),
    (
        "no_neighbour_takes_its_turn_while_a_peak_is_open",
        no_neighbour_takes_its_turn_while_a_peak_is_open,
    ),
];

fn the_counting_allocator_sees_the_heap() {
    let _turn = heap_turn();
    let big = vec![0u8; 64 * 1024 * 1024];
    assert!(
        peak_bytes() >= big.len() as u64,
        "peak {} did not cover a {} byte allocation",
        peak_bytes(),
        big.len()
    );
    drop(big);
}

fn an_anchored_peak_measures_the_rise_not_the_heap_already_held() {
    let held = vec![0u8; 32 * 1024 * 1024];
    let window = anchor_peak();
    assert!(
        window.rise_bytes() < held.len() as u64,
        "the anchor carried the {} bytes already held into the rise",
        held.len()
    );
    let added = vec![0u8; 48 * 1024 * 1024];
    assert!(
        window.rise_bytes() >= added.len() as u64,
        "rise {} did not cover the {} bytes added after the anchor",
        window.rise_bytes(),
        added.len()
    );
    drop(held);
    drop(added);
}

/// A neighbour that frees what it held while a peak is open lowers the live
/// count under the anchor, and the rise reads short by that much: a gate whose
/// other test dropped its own copy of the corpus mid-fold recorded a third of
/// the fold's real peak. So an open measurement keeps every neighbour out.
fn no_neighbour_takes_its_turn_while_a_peak_is_open() {
    let window = anchor_peak();
    let neighbour_got_in = std::thread::spawn(|| try_heap_turn().is_some())
        .join()
        .unwrap();
    assert!(
        !neighbour_got_in,
        "a neighbour took its turn inside an open measurement, so its frees \
         would read as the measured code's own savings"
    );
    drop(window);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--list") {
        if !args.iter().any(|a| a == "--ignored") {
            for (name, _) in CASES {
                println!("{name}: test");
            }
        }
        return;
    }
    let exact = args.iter().any(|a| a == "--exact");
    let filters: Vec<&String> = args.iter().filter(|a| !a.starts_with("--")).collect();
    let selected = CASES.iter().filter(|(name, _)| {
        filters.is_empty()
            || filters.iter().any(|f| {
                if exact {
                    name == f
                } else {
                    name.contains(f.as_str())
                }
            })
    });
    let mut ran = 0;
    for (name, case) in selected {
        case();
        println!("test {name} ... ok");
        ran += 1;
    }
    println!("\ntest result: ok. {ran} passed; 0 failed");
}
