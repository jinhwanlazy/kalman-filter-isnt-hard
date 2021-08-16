# 칼만필터는 어렵지 않아

"[칼만 필터는 어렵지 않아 (저자: 김성필 님)](https://www.hanbit.co.kr/academy/books/book_view.html?p_code=B4956047798)"
에서 소개된 예제 코드를 파이썬으로 구현함
* 원본 matlab 코드를 그대로 베껴쓰기 보단, pythoninc 한 코드로 다시 쓰는데
  집중함.
* 여러 챕터에 반복되어 사용되는 코드는 utils.py 에, 필터 구현은 filters.py 에
  몰아놓음.
* 재사용성을 고려해 각 필터는 class 로 묶고, 모든 필터가 공통된 인터페이스를
  공유하도록 함.
* [공식 자료실](https://www.hanbit.co.kr/support/supplement_list.html)에서
  다운받은 데이터는 `source` directory 를 만들고 그 안에 풀어놓을 것.

# License

Copyright (c) 2021 Jinhwan Choi

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
