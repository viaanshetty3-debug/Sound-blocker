# Contributing

Thanks for your interest. This project is a simulation and unbuilt design, so the most useful
contributions are the ones that test it against reality.

## Most wanted
1. **Bench measurements.** Run `python3 tools/timing_tool.py measure` on a Teensy and open an issue with the output.
2. **Corrections.** If a claim, formula or number is wrong, say so. Please include the check you ran.
3. **Hardware build reports**, including what failed.

## Ground rules
* No unsupported figures. Every number must come from code in this repo, a datasheet, or a measurement you describe.
* Label simulation results as simulation. Do not describe them as real-world performance.
* Run the tests before opening a pull request:
  `python3 -m unittest discover -s tests -p "test_*.py"`
