- 方法和函数是不同的，通过"_"可以将方法转换成函数,但是"_"不是必须的！
```scala
import scala.math._
val fun = ceil _
```
- “高阶函数”是接收函数参数的函数，或者产生函数的函数。
    - map
    - foreach
    - filter
    - reduceLeft
    - sortWith
- 闭包
```scala
 def mulBy(factor:Double)=(x:Double) => factor *x
 val mul3 = mulBy(3)
 println(mul3(9))
```
- SAM 转换，针对2.12才有的特性。
- 柯里化：将接受两个参数的函数变成接受一个参数的函数的过程！返回的函数以原来第二个参数作为单参数。
```scala
def mulOneAtATime(x:Int)(y:Int) = x*y
val mulOneAtATime = (x:Int) =>((y:Int)=> x*y)
```
- 控制抽象：即定义一个看上去是关键字的函数，比如在scala中定义untile关键字(当条件成立时退出)
```scala
object Main {
// : => 实际上是一个控制抽象定义
  def untile(condition: => Boolean)(
  ): Unit = {
    if (!condition) {
      block
      untile(condition)(block)
    }
  }

  def main(args: Array[String]): Unit = {
    var x = 10
    untile(x == 0) {
      x -= 1
      println(x)
    }
  }
}
```