# 运算符

- ``用作标识符转义
- 中置表达式是二元的操作符，在class中直接定义“操作符”同名函数

```scala
import java.awt.Point

class OrderedPoint(x: Int, y: Int) extends java.awt.Point with scala.math.Ordered[Point] {
  override def compare(that: Point): Int = if (y < that.y && x <= that.x) -1 else 1
  def + (that: Point) = new OrderedPoint(x+that.x,y+that.y)
  def - (that: Point) = new OrderedPoint(x-that.x,y-that.y)
}
```

- 前置表达式是一元操作符，常见将+，-，!，~定义成前置操作符。前置表达式被调用时转换成“unary_操作符”被调用
  
```scala

class OrderedPoint(x: Int, y: Int) extends java.awt.Point (x, y)with scala.math.Ordered[Point] {
  override def compare(that: Point): Int = if (y < that.y && x <= that.x) -1 else 1
  def unary_- = new OrderedPoint(-x, -y)
}
```

- 复值操作符，如+=，-=，*=等。定义类似中置操作符。
- 优先级
  
    - 操作符的优先级由首字符决定
  
    - 中置操作优先级高于后置
- 结合性：均为左结合（从左到右求值），除了

    - ：开头的操作符
    - 赋值操作符？？？？ 好像也是左结合的
- apply方法和update方法
    - OjectName(arg1,arg2,......)出现在=右侧，等价为OjectName.apply(arg1,arg2,......)
    - OjectName(arg1,arg2,......)出现在=左侧，等价为OjectName.update(arg1,arg2,......,value)
- unapply方法:主要用在case模式匹配中,返回值为Option[T]或者Option[(T,U)]或者Boolean
  
  ```scala
  class Money(val value: Double, val country: String) {}
  object Money {
    def apply(value: Double, country: String) : Money = new Money(value, country)

    def unapply(money: Money): Option[(Double, String)] = {
        if(money == null) {
            None
        } else {
            Some(money.value, money.country)
        }
    }

    def testUnapply() = {
        val money = Money(10.1, "RMB")
        money match {
            case Money(num, "RMB") =>  println("RMB: " + num) //这里调用了unapply方法,因此num可以取到值为10.1 等于调用了Money.unapply(money)
            case _ => println("Not RMB!")
        }
    }
    ```
- 通过unapplySeq方法可以返回任意序列的匹配，unapplySeq和unapply不能同时定义。

```scala

object OrderedPoint {
//  def unapply(arg: OrderedPoint): Option[(Int, Int)] = if (arg == null) None else Some((arg.x, arg.y))

  def unapplySeq(arg: String): Option[Seq[String]] = Some(arg.split(" "))
}

object Main {

  def main(args: Array[String]): Unit = {
    val OrderedPoint(a,b,c,d)="dfadf dfadf dfadf dfadsf"
    println(a)
    println(b)
    println(c)
    println(d)
  }
}
```

-  动态调用
  
```scala

import scala.collection.mutable
import scala.language.dynamics


class DynamicObj extends Dynamic {

  //定义一个方法类型 CallFun，它接收 Int 类型参数，并返回 String 类型 ，这个有点像 C# 中的 delegate
  type CallFun = String => Unit

  //Map对象，存放方法对象和字段
  private val functions =mutable.HashMap.empty[String,CallFun].withDefault{ key => throw new NoSuchFieldError(key) }
  private val fields = mutable.HashMap.empty[String, Any].withDefault { key => throw new NoSuchFieldError(key) }

  //获取key对应的value值
  def selectDynamic(key: String) = fields(key)

  /**
  更新key对应的value
  这里做了一个判断，如果key以call字符串开头，我们认为是args是CallFun类型
    */
  def updateDynamic(key: String)(args: Any): Unit ={
    args match {
      case x if key.startsWith("call") => functions(key) = x.asInstanceOf[CallFun]
      case _ => fields(key) = args
    }
  }

  //这个就是用来动态执行方法的
  def applyDynamic(key: String)(arg:String) = {
    functions(key)(arg)
  }
}

object DynamicObj {
  def main(args: Array[String]): Unit = {
    val dynamicObj = new DynamicObj
    dynamicObj.Name = "linqing"
    dynamicObj.call_SaySomething = (some:String) => println(some)
    println(dynamicObj.Name)
    dynamicObj.call_SaySomething("hello")
  }
}
```