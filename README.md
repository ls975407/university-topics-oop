# 平面多連桿模擬程式

> 大學**物件導向**課程的期末專題 簡報: [平面機構模擬 簡報影片](https://www.youtube.com/watch?v=4Rr0mEYFWQo)

<table style="border: none;"><tr style="border: none;">
<td style="border: none;">

用於模擬 **機構學**多連桿系統的運動情形

提供給機構設計師的**電腦輔助軟體**，

類似軟體如

[ARTAS   2D動態分析軟體](https://www.simweb.com.tw/contents/zh-tw/p66311_ARTAS.html)
    
[Linkage 平面機構設計軟體](https://blog.rectorsquid.com/linkage-mechanism-designer-and-simulator/)

</td>
<td style="border: none;">

![movie](short.gif)

</td>
</tr></table>


## 物件導向設計細節
> 以下用**top-down程式設計**順序說明，括弧英文對應程式碼的物件名稱。

![類別階層](class_graph.png)

### 1. 連桿(Linkage)
為一個**實體物件**，為多連桿系統，

有馬達、轉軸等特殊功能點組成，在設定點間的幾何限制、馬達旋轉的頻率、模擬時間單位後，即可描繪出連桿運動的情形。

新增功能點時，會根據 機構學檢查自由度，以利計算出唯一的結果

### 2. 功能點(PointBC)
為一個**虛擬物件**，為馬達、轉軸或固定點。

此程式實作三個物件

    PointSE:   端點
	
    PointRot:  馬達
	
    PointMov:  轉軸

### 3. 端點(PointSE)
為一個**實體物件**，為固定在物體上的點，

本身可以是觀察點(位置不變)、或是以其他功能點參考決定位置。

需要設定幾何條件(ConditionSE)來計算位置。

### 4. 端點的幾何條件(ConditionSE)
為一個**虛擬物件**，為計算端點位置的數學方法。

此程式實作兩個 **子類別**

    CRef_p2_fix:   位置不變(觀察點)
	
    CRef_p2_scale: 根據兩個功能點之間的固定位置計算(固定在會動的連桿上)

### 5. 旋轉點(PointRot)
為一個**實體物件**，即驅動源，如馬達，需與端點或馬達相連(由連桿物件檢查)。

需要設定旋轉條件(ConditionSE)來計算位置。

### 6. 旋轉點的幾何條件(ConditionRot)
為一個**虛擬物件**，為計算旋轉點位置的數學方法。

此程式實作一個 **子類別**

    CRot_point_linear: 等速等距繞著功能點旋轉(線性馬達)

### 7. 移動點(PointMov)
為一個**實體物件**，即轉軸。

需要設定移動條件(ConditionMov)來計算位置。
	
根據不同移動條件，機構自由度會受影響(由連桿物件檢查) 。

### 8. 移動點的幾何條件(ConditionMov)
為一個**虛擬物件**，為計算移動點位置的數學方法。

此程式實作兩個 **子類別**

    CMov_p2: 和兩個功能點間，分別各有一個固定的距離(連桿機構)

    CMov_p1: 和一個虛擬的線間，有一個固定的距離(滑塊機構)


## 模擬細節
> 第一次迴圈可以計算的是固定位置的觀察點，
>
> 第二次迴圈可以計算的是以觀察點為條件的旋轉點，
>
> 後面每次迴圈計算的功能點和數量會依據連桿設計的方式而有不同。

透由連桿物件執行模擬，先將所有功能點都設定為尚未計算。

跑迴圈，發現至少有一個功能點可以被計算，

則重複跑迴圈，直到所有點計算完成即成功；否則失敗。


## 貢獻者

機構模擬: 劉丁豪

介面設計: 林郁恆 

開發環境: [Qt/C++](https://www.qt.io/)

專題簡報: [平面機構模擬 簡報影片](https://www.youtube.com/watch?v=4Rr0mEYFWQo)

(只上傳機構模擬相關程式設計以利交流)

## 相關連結

維基百科: [平面四連桿機構](https://zh.wikipedia.org/wiki/%E5%B9%B3%E9%9D%A2%E5%9B%9B%E6%9D%86%E6%9C%BA%E6%9E%84)
