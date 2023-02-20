from ament_pycodestyle.main import generate_pycodestyle_report
import pytest

# RUN directly: python3 -m pytest ./src/robot_control/test/test_pep8_codestyle.py; but you need to define paths as ['./src/robot_control']


@pytest.mark.linter
def test_pep8_codestyle():
    paths = ['.']

    report = generate_pycodestyle_report(None, paths, None, 200)

    n_errors = len(report.errors)

    print('Found %d code style errors / warnings:' % n_errors)
    # print(report.errors)

    if (n_errors != 0):
        for error in report.errors:
            print(error)
            # Nicer printing:
            # print(error['path'] + ':' + str(error['row']) + ':' +
            #       str(error['column']) + ': ' + str(error['error_code']) + ' ' +
            #       str(error['error_message']) + '\n"' + str(error['source_line']) +
            #       '"\n')

    assert n_errors == 0, \
        '----Found %d code style errors / warnings:\n' % n_errors + \
        '\n'.join(report.errors)
