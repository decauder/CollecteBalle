from ament_pycodestyle.main import generate_pycodestyle_report
import pytest

# RUN: python3 -m pytest ./src/top_camera/test/test_pep8_codestyle.py

@pytest.mark.linter
def test_pep8_codestyle():
    paths = ['src/top_camera/top_camera']

    report = generate_pycodestyle_report(None, paths, None, 200)

    n_errors = len(report.errors)

    print('Found %d code style errors / warnings:' % n_errors)
    # print(report.errors)

    if (n_errors != 0):
        for error in report.errors:
            # print( error )
            print(error['path'] + ':' + str(error['row']) + ':' +
                  str(error['column']) + ': ' + error['error_code'] + ' ' +
                  error['error_message'] + '\n"' + error['source_line'] +
                  '"\n')

    assert n_errors == 0, \
        '----Found %d code style errors / warnings:\n' % n_errors + \
        '\n'.join(report.errors)
