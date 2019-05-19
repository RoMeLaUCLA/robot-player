# Daniel Sun 11 Apr 2019 
# robot-player
class DxlReadDataError(Exception):
    """
    Raise this when data packets are incorrect
    """
    pass


def read_data():
    print("beginning of function")

    print('reading data')

    raise DxlReadDataError

    print('ending function')



if __name__ == '__main__':
    try:
        read_data()
    except DxlReadDataError:
        print("catching exception here")
    finally:
        print('ending script')

    print('script is over')