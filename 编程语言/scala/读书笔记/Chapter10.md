# 特质
- JAVA的接口：只包含抽象方法、静态方法、默认方法、不能包含字段
- scala的特质：允许抽象方法和具体方法、能够包含字段
- 所有Java接口都能当作特质使用
- 创建对象时，可以通过with关键字为对象混入特质（前提条件是，对象时abstract类型，定义时混入了一个没有实现的trait，当定义这个对象时能够混入trait的子类）
```scala
trait Speaker {
  def say
}

trait ChineseSpeaker extends Speaker{
  override def say =println("你好")
}

trait EnglishSpeaker extends Speaker{
  override def say: Unit = println("hello")
}

abstract class Person extends Speaker  {
  def talk = say
}

object Person {
  def main(args: Array[String]): Unit = {
    val chinese = new Person with ChineseSpeaker
    val english = new Person with EnglishSpeaker
    chinese.talk
    english.talk
  }
}

```
- 特质叠加，多个特质有同名方法时，最后一个with的特质的方法被执行，使用super.menthod可以调用上一个特质的方法。super[特质].menthod
- 特质的具体字段是被作为子类字段的，而是超类字段。抽象字段必须在实现类中被重启写（重新定义）。
- 构造器初始化顺序：
    - 超类构造器
    - 特质构造器（从左到右），先执行特质的父类构造器，如果多个特质有共同父类，那么只有一个父类被执行
    - 子类构造器
- 初始化特质中的抽象字段时，需要考虑字段的初始化化顺序。特质不支持构造器参数，因此可能出现子类应用未初始化字段。
    - 提前定义
    - 懒加载
- 扩展类特质、自身类型：此时特质只能混入到共同基类的Class中